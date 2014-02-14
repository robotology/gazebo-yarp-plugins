/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#ifndef _GAZEBO_YARP_FORCETORQUE_PLUGIN_HH_
#define _GAZEBO_YARP_FORCETORQUE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <yarp/os/Network.h>
#include <yarp/dev/PolyDriver.h>
#include <string>

namespace gazebo
{
    namespace sensors {
        class ForceTorqueSensor;
    }
    
    class GazeboYarpForceTorque : public SensorPlugin
    {
    public:
        GazeboYarpForceTorque();
        virtual ~GazeboYarpForceTorque();

        virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    private:
        sensors::ForceTorqueSensor* parentSensor;
        yarp::os::Network _yarp;
        yarp::dev::PolyDriver _forcetorque_wrapper;
        yarp::dev::PolyDriver _forcetorque_driver;
        
        std::string _sensorName;
        
        virtual void Init();
    };
}

#endif
