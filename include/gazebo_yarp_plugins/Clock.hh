/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef GAZEBOYARP_CLOCKPLUGIN_HH
#define GAZEBOYARP_CLOCKPLUGIN_HH

#include <gazebo/gazebo.hh>
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>

namespace gazebo
{
    class GazeboYarpClock : public SystemPlugin
    {
    public:
        GazeboYarpClock();
        virtual ~GazeboYarpClock();

        virtual void Load(int _argc = 0, char **_argv = NULL);
        
        void GazeboYarpClockLoad(std::string world_name);
        
        void ClockUpdate();
        
    private:
        virtual void Init() {};
        
        yarp::os::Network _yarp;

        std::string port_name;
        
        yarp::os::BufferedPort<yarp::os::Bottle> port;  
        
        gazebo::event::ConnectionPtr time_update_event_;

        gazebo::event::ConnectionPtr load_gazebo_yarp_clock;
        
        gazebo::physics::WorldPtr world_;
        
    };
};

#endif 
