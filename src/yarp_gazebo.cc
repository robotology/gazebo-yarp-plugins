/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia iCub Facility
 * Authors: Lorenzo Natale and Paul Fitzpatrick and Mingo Enrico
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <yarp/os/Network.h>
#include <gazebo_world.h>
#include <fakebot.h>

#define ms(X) (X * 1000.0)
#define toRad(X) (X*M_PI/180.0)
#define ROBOT_NAME "COMAN"

namespace gazebo
{
    class yarp_gazebo : public ModelPlugin
    {
    public:
        yarp_gazebo() : _yarp()
        {

        }

        void Init()
        {
            std::cout<<"*** GAZEBO 2 YARP ***"<<std::endl;
            if (!_yarp.checkNetwork())
                std::cout<<"Sorry YARP network does not seem to be available, is the yarp server available?"<<std::endl;
            else
                std::cout<<"YARP Server found!"<<std::endl;
        }

         ~yarp_gazebo()
        {
            _driver.close();
            std::cout<<"Goodbye!"<<std::endl;
        }

        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            this->_robot = _parent;

            _dT = ms(this->_robot->GetWorld()->GetPhysicsEngine()->GetUpdatePeriod());
            std::cout<<"Simulation Time Step: "<<_dT<<" [ms]"<<std::endl;

            gazebo_world::setModel(this->_robot);
            std::cout<<"Robot Name: "<<_robot->GetName()<<std::endl;
            std::cout<<"# Joints: "<<_robot->GetJoints().size()<<std::endl;
            std::cout<<"# Links: "<<_robot->GetLinks().size()<<std::endl;

            yarp::dev::Drivers::factory().add(new yarp::dev::DriverCreatorOf<yarp::dev::fakebot>
                                              ("fakebot", "controlboard", "fakebot"));
            _parameters.put("device", "controlboard");
            _parameters.put("subdevice", "fakebot");
            _parameters.put("name", "/fakebot/head");
            _parameters.put("dT", _dT);


            _driver.open(_parameters);
            if (!_driver.isValid())
               fprintf(stderr, "Device did not open\n");

            printf("Device initialized correctly, now sitting and waiting\n");

            std::cout<<"LOADED GAZEBO 2 YARP PLUGIN!"<<std::endl;
        }

    private:
        /**
          * Simulation Time Step in ms
          */
        double _dT;
        physics::ModelPtr _robot;
        yarp::os::Network _yarp;
        yarp::dev::PolyDriver _driver;
        yarp::os::Property _parameters;
        event::ConnectionPtr updateConnection; // Pointer to the update event connection
    };

    GZ_REGISTER_MODEL_PLUGIN(yarp_gazebo)
}
