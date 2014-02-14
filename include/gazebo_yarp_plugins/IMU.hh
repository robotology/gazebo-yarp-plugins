/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#ifndef GAZEBOYARP_IMU_HH
#define GAZEBOYARP_IMU_HH

#include <gazebo/gazebo.hh>
#include <yarp/os/Network.h>
#include <yarp/dev/PolyDriver.h>
#include <string>

namespace gazebo
{
    namespace sensors {
        class ImuSensor;
    }
    
    class GazeboYarpIMU : public SensorPlugin
    {
    public:
        GazeboYarpIMU();
        virtual ~GazeboYarpIMU();

        virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    private:
        sensors::ImuSensor* parentSensor;
        yarp::os::Network _yarp;
        yarp::os::Property _parameters; 
        yarp::dev::PolyDriver _imu_driver;
        std::string _sensorName;
        
        virtual void Init();
    };
}

#endif
