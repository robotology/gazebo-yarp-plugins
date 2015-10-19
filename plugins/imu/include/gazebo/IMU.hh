/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#ifndef GAZEBOYARP_IMU_HH
#define GAZEBOYARP_IMU_HH

#include <gazebo/common/Plugin.hh>

#include <yarp/dev/PolyDriver.h>

#include <string>


namespace gazebo
{
    namespace sensors {
        class ImuSensor;
    }

    /// \class GazeboYarpIMU
    /// Gazebo Plugin emulating the yarp imu device in Gazebo.
    ///
    /// This plugin instantiate a yarp imu driver for the Gazebo simulator
    /// and instantiate a network wrapper (provided by yarp::dev::ServerInertial)
    /// to expose the sensor on the yarp network.
    ///
    /// It can be configurated using the yarpConfigurationFile sdf tag,
    /// that contains a Gazebo URI pointing at a yarp .ini configuration file
    /// containing the configuration parameters of the controlBoard
    ///
    /// The parameter that the yarpConfigurationFile must contain are:
    ///  <TABLE>
    ///  <TR><TD> name </TD><TD> Port name to assign to the wrapper to this device. </TD></TR>
    ///  <TR><TD> period </TD><TD> Update period (in s) of yarp port that publish the measure. </TD></TR>
    ///  </TABLE>
    /// If the required parameters are not specified, their value will be the
    /// default one assigned by the yarp::dev::ServerInertial wrapper .
    ///
    class GazeboYarpIMU : public SensorPlugin
    {
    public:
        GazeboYarpIMU();
        virtual ~GazeboYarpIMU();
        virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    private:
        yarp::os::Property m_parameters; 
        yarp::dev::PolyDriver m_imuDriver;
        std::string m_sensorName;
    };
}

#endif
