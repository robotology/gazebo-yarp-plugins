/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#ifndef _FAKEBOT_IMU_PLUGIN_HH_
#define _FAKEBOT_IMU_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <yarp/os/Network.h>
#include <yarp/dev/PolyDriver.h>

namespace gazebo
{
    class fakebotIMUplugin : public SensorPlugin
    {
    public:
        fakebotIMUplugin();
        virtual ~fakebotIMUplugin();

        virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    private:
        sensors::ImuSensorPtr parentSensor;
        event::ConnectionPtr updateConnection;
        yarp::os::Network _yarp;
        yarp::os::Port _p;
        yarp::os::Bottle _bot;

        virtual void Init();
        virtual void OnUpdate();

    };
}

#endif
