/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#ifndef GAZEBOYARP_FORCETORQUE_HH
#define GAZEBOYARP_FORCETORQUE_HH

#include <gazebo/common/Plugin.hh>

#include <yarp/dev/PolyDriver.h>

#include <string>

namespace yarp {
    namespace dev {
        class IMultipleWrapper;
    }
}
namespace gazebo
{
    namespace sensors {
        class ForceTorqueSensor;
    }

    /// \class GazeboYarpForceTorque
    /// \brief Gazebo Plugin emulating the yarp device exposing a 6 axis force-torque sensor.
    ///
    /// This plugin instantiate a yarp 6-axis force torque sensor driver for the Gazebo simulator
    /// and instantiate a network wrapper (provided by yarp::dev::AnalogWrapper)
    /// to expose the sensor on the yarp network.
    ///
    /// It can be configurated using the yarpConfigurationFile sdf tag,
    /// that contains a Gazebo URI pointing at a yarp .ini configuration file
    /// containing the configuration parameters of the sensor.
    ///
    /// The parameter that the yarpConfigurationFile should contain are:
    ///  | Parameter name | Type   | Units | Default Value | Required | Description | Notes |
    ///  |:--------------:|:------:|:-----:|:-------------:|:--------:|:-----------:|:-----:|
    ///  |  name          | string |   -   | See  yarp::dev::AnalogWrapper  | No      | Robot name, used to form the prefix for the opened ports. | |
    ///  |  period        | int    |   ms  | See  yarp::dev::AnalogWrapper   | No      | Update period (in ms) of yarp port that publish the measure. | |
    ///
    /// If this parameters are not specified, their value will be the
    /// default one assigned by the yarp::dev::AnalogWrapper .
    ///
    /// The port opened by the yarp::dev::AnalogWrapper will be:
    ///  | Port name |  Description |
    ///  |:----------:|:------------:|
    ///  | {name}    | Port streaming the measure |
    ///  | {name}/rpc:i | RPC Port |
    ///
    ///
    class GazeboYarpForceTorque : public SensorPlugin
    {
    public:
        GazeboYarpForceTorque();
        virtual ~GazeboYarpForceTorque();
        
        virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    private:
        yarp::dev::PolyDriver m_forcetorqueWrapper;
        yarp::dev::IMultipleWrapper* m_iWrap;
        yarp::dev::PolyDriver m_forceTorqueDriver;
        
        std::string m_sensorName;
    };
}

#endif
