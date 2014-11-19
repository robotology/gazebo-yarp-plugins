/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef GAZEBOYARP_CLOCKPLUGIN_HH
#define GAZEBOYARP_CLOCKPLUGIN_HH

#include <gazebo/common/Plugin.hh>

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

        void gazeboYarpClockLoad(std::string world_name);

        void clockUpdate();

        void clockPause();

        void clockContinue();

        void clockStep(unsigned steps=1);

    private:
        std::string m_portName;
        yarp::os::BufferedPort<yarp::os::Bottle> m_port;

        gazebo::event::ConnectionPtr m_timeUpdateEvent;

        gazebo::event::ConnectionPtr m_worldCreatedEvent;

        gazebo::physics::WorldPtr m_world;

    };
}

#endif
