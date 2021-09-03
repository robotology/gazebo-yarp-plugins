/*
 * Copyright (C) 2014 CoDyCo
 * Author: Jorh, Francesco Romano
 * email:  daniele.pucci@iit.it, francesco.romano@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <stdio.h>
#include <math.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;


const double pi = 3.141592654;
const double g = 9.80;

int main(int argc, char **argv)
{

    Network yarp;

    Property params;
    params.fromCommand(argc, argv);

    std::string robotName;
    
    if (!params.check("robot"))
    {
        robotName = "doublePendulumGazebo";
//         yError("Please specify the name of the robot\n");
//         yError("--robot name (e.g. icub)\n");
//         return -1;
    }
    else {
        robotName=params.find("robot").asString().c_str();
    }
    std::string remotePorts="/";
    remotePorts+=robotName;
    remotePorts+="/body";

    std::string localPorts="/torqueModule";

    Property options;
    options.put("device", "remote_controlboard");
    options.put("local", localPorts.c_str());   //local port names
    options.put("remote", remotePorts.c_str());         //where we connect to

    // create a device
    PolyDriver robotDevice(options);
    if (!robotDevice.isValid()) {
        yError("Device not available.  Here are the known devices:\n");
        yError("%s", Drivers::factory().toString().c_str());
        return 0;
    }

    IControlMode *controlMode;
    ITorqueControl *torque;
    IPositionControl *positionControl;
    IEncoders *encoders;

    bool ok;
    ok = robotDevice.view(controlMode);
    ok = ok && robotDevice.view(torque);
    ok = ok && robotDevice.view(positionControl);
    ok = ok && robotDevice.view(encoders);
    
    if (!ok) {
        yError("Problems acquiring interfaces\n");
        return 0;
    }
    
    BufferedPort<Bottle> *speedInput = new BufferedPort<Bottle>();
    if (!speedInput || !speedInput->open((localPorts + "/speeds:i").c_str())) {
        yError("Could not open port speed\n");
        return false;
    }

    yDebug("Trying to connect: %s to %s...", (remotePorts + "/analog/speed").c_str(), (localPorts + "/speeds:i").c_str());
    Network::connect((remotePorts + "/analog:o/speed").c_str(), (localPorts + "/speeds:i").c_str());
    yDebug("hopefully ok \n");

    //parameters
    const double m1  = 1;
    const double m2  = 1;
    const double l1  = 0.5;
    const double l2  = 0.5;
    const double a1  = 1;
    const double Il1 = 1.0/12.0;
    const double Il2 = 1.0/12.0;

    controlMode->setTorqueMode(0);
    controlMode->setTorqueMode(1);
    
    double tau1 = 0;
    double tau2 = 0;
    
    double readTau1 = 0;
    double readTau2 = 0;
    
    double grav1 = 0;
    double grav2 = 0;
    
    double cH = 0;
    double cTerm1 = 0;
    double cTerm2 = 0;
    
    double m11 = 0;
    double m12 = 0;
    double m22 = 0;
    
    double q1 = 0;
    double q2 = 0;
    double dq1 = 0;
    double dq2 = 0;
    
    yarp::sig::Vector qDes(2);
    qDes(0) = 0.0;
    qDes(1) = 0.7;
    
    yarp::sig::Matrix kp(2,2);
    kp.zero();
    kp(0, 0) = 0.1; kp(1, 1) = 0.1;
    yarp::sig::Matrix kd(2,2);
    kd.zero();
    kd(0, 0) = 1; kd(1, 1) = 1;

     
    while(1) {
        
        encoders->getEncoder(0, &q1);
        encoders->getEncoder(1, &q2);
        
        q1 = q1 / 180 * pi; //convert in radiants
        q2 = q2 / 180 * pi;
        
        Bottle *speed = speedInput->read(false); //this is in radiants per second
        if (speed) {
            yarp::os::Value val1 = speed->get(0);
            yarp::os::Value val2 = speed->get(1);
            
            if (!val1.isNull())
                dq1 = val1.asFloat64();
            if (!val2.isNull())
                dq2= val2.asFloat64();
        }
        
        yarp::sig::Vector q(2);
        q(0) = q1;
        q(1) = q2;
        
        yarp::sig::Vector dq(2);
        dq(0) = dq1;
        dq(1) = dq2;
        
        grav1 = (m1 * l1 + m2 * a1) * g * cos(q1) + m2 * l2 * g * cos(q1 + q2);
        grav2 = m2 * l2 * g * cos(q1 + q2);
        
        cH = -m2 * a1 * l2 * sin(q2);
        cTerm1 = cH * dq2 * dq1 + cH * (dq1 + dq2) * dq2;
        cTerm2 = -cH * dq1 * dq1;
         
        m11 = Il1 + m1 * l1 * l1 + Il2 + m2 * (a1 * a1 + l2 * l2 + 2 * a1* l2 * cos(q2));
        m12 = Il2 + m2 * (l2 * l2 + a1* l2 * cos(q2));
        m22 = Il2 + m2 * l2 * l2;
        
        yarp::sig::Matrix M(2,2);
        M(0,0) = m11; 
        M(0,1) = M(1,0) = m12;
        M(1,1) = m22;
        
        yarp::sig::Vector Cdq(2);
        Cdq(0) = cTerm1;
        Cdq(1) = cTerm2;
        
        yarp::sig::Vector grav(2);
        grav(0) = grav1;
        grav(1) = grav2;
        
        yarp::sig::Vector tau(2);
        
//         tau =   grav + Cdq - M * (kd * dq + kp * (q - qDes)); // computed torque
        tau =   grav - (kd*dq + kp*(q-qDes)); //PD + gravity compensation
        
        yarp::sig::Vector error(2);
        error = q-qDes;
        
        yError("Error: %lf * %lf      Torque:  %lf * %lf       Vel:  %lf * %lf\n", error(0), error(1), tau(0), tau(1), dq(0), dq(1));
        
        torque->setRefTorques(tau.data());
        
        torque->getTorque(0, &readTau1);
        torque->getTorque(1, &readTau2);
        
        //check lyapunov function
        yarp::sig::Matrix dqMatrix(2, 1);
        dqMatrix.setSubcol(dq, 0, 0);
        yarp::sig::Matrix eMatrix(2, 1);
        eMatrix.setSubcol(error, 0, 0);
        
        yarp::sig::Matrix vLyapunov(1, 1);
        vLyapunov = 0.5 * (dqMatrix.transposed() * M) * dqMatrix + 0.5 * eMatrix.transposed() * kp * eMatrix;
        yInfo("Lyapunov Value: %lf\n", vLyapunov(0, 0));
        
        
        yarp::os::Time::delay(0.01);
    }
    

    speedInput->close();
    delete speedInput;
    robotDevice.close();
    
    return 0;


}
