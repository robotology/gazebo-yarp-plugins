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
#define ROBOT_NAME "COMAN"

namespace gazebo
{
    class yarp_gazebo : public WorldPlugin
    {
    public:
        yarp_gazebo(): WorldPlugin()
          ,_yarp()
        {
            std::cout<<"*** GAZEBO 2 YARP ***"<<std::endl;
            if (!_yarp.checkNetwork())
                std::cout<<"Sorry YARP network does not seem to be available, is the yarp server available?"<<std::endl;
            else
                std::cout<<"YARP Server found!"<<std::endl;
        }

         ~yarp_gazebo()
        {
            std::cout<<"Goodbye!"<<std::endl;
            _driver.close();
        }

        void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
        {
            this->_world = _parent;

            gazebo_world::setWorld(_world);

            _dT = ms(_world->GetPhysicsEngine()->GetUpdatePeriod());
            std::cout<<"Simulation Time Step: "<<_dT<<" [ms]"<<std::endl;


            this->_robot = _world->GetModel(ROBOT_NAME);
            gazebo_world::setModel(_robot);
            std::cout<<"Robot Name: "<<_robot->GetName()<<std::endl;
            std::cout<<"# Joints: "<<_robot->GetJoints().size()<<std::endl;
            std::cout<<"# Links: "<<_robot->GetLinks().size()<<std::endl;

            yarp::dev::Drivers::factory().add(new yarp::dev::DriverCreatorOf<yarp::dev::fakebot>
                                              ("fakebot", "controlboard", "fakebot"));
            _parameters.put("device", "controlboard");
            _parameters.put("subdevice", "fakebot");
            _parameters.put("name", "/fakebot");
            _parameters.put("dT", _dT);


            _driver.open(_parameters);
            if (!_driver.isValid())
               fprintf(stderr, "Device did not open\n");

            printf("Device initialized correctly, now sitting and waiting\n");

            std::cout<<"LOADED GAZEBO 2 YARP PLUGIN!"<<std::endl;

        }

    private:
        physics::WorldPtr _world;
        /**
          * Simulation Time Step in ms
          */
        double _dT;
        physics::ModelPtr _robot;
        yarp::os::Network _yarp;
        yarp::dev::PolyDriver _driver;
        yarp::os::Property _parameters;
    };

    GZ_REGISTER_WORLD_PLUGIN(yarp_gazebo)
}
