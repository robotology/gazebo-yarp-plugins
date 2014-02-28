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

#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <stdio.h>
#include <cmath>

using namespace yarp::os;
using namespace yarp::dev;


const double pi = 3.141592654;
const double g = 9.80;

int main(int argc, char **argv)
{
//     ResourceFinder *rf = new ResourceFinder;
//     
//     rf->setVerbose(true);
//     rf->setDefaultConfigFile("default.ini");         //default config file name.
//     rf->setDefaultContext("TorqueControl/conf"); //when no parameters are given to the module this is the default context
//     rf->configure("ICUB_ROOT",argc,argv);
//     
//     if (rf->check("help"))
//     {
//         std::cout<< "Possible parameters" << std::endl << std::endl;
//         std::cout<< "\t--context          :Where to find a user defined .ini file within $ICUB_ROOT/app e.g. /adaptiveControl/conf" << std::endl;
// //        std::cout<< "\t--from             :Name of the file.ini to be used for calibration." << std::endl;
//         std::cout<< "\t--rate             :Period used by the module. Default set to 10ms." << std::endl;
//         std::cout<< "\t--robot            :Robot name (icubSim or icub). Set to icub by default." << std::endl;
//         std::cout<< "\t--local            :Prefix of the ports opened by the module. Set to the module name by default, i.e. adaptiveControl." << std::endl;
//         return 0;
//     }
//     
//     Network yarp;
//     
//     if (!yarp.checkNetwork())
//     {
//         std::cerr << "Sorry YARP network is not available\n";
//         return -1;
//     }
//     
//     //Creating the module
//     torqueController::TorqueControllerModule module();
//     return module.runModule(*rf);

    Network yarp;

    Property params;
    params.fromCommand(argc, argv);

    std::string robotName;
    
    if (!params.check("robot"))
    {
        robotName = "doublePendulumGazebo";
//         fprintf(stderr, "Please specify the name of the robot\n");
//         fprintf(stderr, "--robot name (e.g. icub)\n");
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
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
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
        printf("Problems acquiring interfaces\n");
        return 0;
    }
    
       BufferedPort<Bottle> *speedInput = new BufferedPort<Bottle>();
        if (!speedInput || !speedInput->open(("/" + localPorts + "/speeds:i").c_str())) {
            fprintf(stderr, "Could not open port speed\n");
            return false;
        }
        Network::connect((remotePorts + "/analog/speeds:o").c_str(), ("/" + localPorts + "/speeds:i").c_str());


    //parameters
    const double m1 = 1;
    const double m2 = 1;
    const double l1 = 0.5;
    const double l2 = 0.5;
    const double a1 = 1;
    

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
                dq1 = val1.asDouble();
            if (!val2.isNull())
                dq2= val2.asDouble();              
        }
        
        
        
        grav1 = (m1 * l1 + m2 * a1) * g * cos(q1) + m2 * l2 * g * cos(q1 + q2);
        grav2 = m2 * l2 * g * cos(q1 + q2);
        
        cH = -m2 * a1 * l2 * sin(q2);
        cTerm1 = cH * dq2 * dq1 + cH * (dq1 + dq2) * dq2;
        cTerm1 = -cH * dq1 * dq1;
         
        tau1 = grav1 + cTerm1;
        tau2 = grav2 + cTerm2;
        
        torque->setRefTorque(0, tau1);
        torque->setRefTorque(1, tau2);
        
        torque->getTorque(0, &readTau1);
        torque->getTorque(1, &readTau2);
        
        printf("Joint 0: written %lf\tread %lf\t\tJoint 1: written %lf\tread %lf\n", tau1, readTau1, tau2, readTau2);
        
        yarp::os::Time::delay(0.01);
    }
    

    speedInput->close();
    delete speedInput;
    robotDevice.close();
    
    return 0;


}
