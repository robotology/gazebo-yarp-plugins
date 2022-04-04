/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#ifndef GAZEBOYARP_IMU_HH
#define GAZEBOYARP_IMU_HH

#include <gazebo/common/Plugin.hh>

#include <yarp/dev/IMultipleWrapper.h>
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
    /// and two network wrapper, the yarp::dev::MultipleAnalogSensorsServer and yarp::dev::ServerInertial
    /// for maintaining backward compatibility.
    /// It is supported also the legacy behaviour of the device-subdevice
    /// mechanism; in this case only the ServerInertial is instantiated.
    ///
    /// It can be configured using the yarpConfigurationFile sdf tag,
    /// that contains a Gazebo URI pointing at a yarp .ini configuration file
    /// containing the configuration parameters of the controlBoard.
    ///
    /// Here is an example of ini file for load the plugin:
    ///
    ///  [include "gazebo_icub_robotname.ini"]
    ///  [WRAPPER]
    ///  device multipleanalogsensorsserver
    ///  name /${gazeboYarpPluginsRobotName}/head/inertials
    ///  period 10
    ///
    ///  [ADDITIONAL_WRAPPER]
    ///  device inertial
    ///  name /${gazeboYarpPluginsRobotName}/inertial
    ///  period 0.01
    ///
    ///  [IMU_DRIVER]
    ///  device gazebo_imu
    ///
    /// On the other hand, here is an example of ini file that exploits the legacy behaviour
    ///
    ///  [include "gazebo_icub_robotname.ini"]
    ///  name /${gazeboYarpPluginsRobotName}/inertial
    ///  period 0.01
    ///  device inertial
    ///  subdevice gazebo_imu
    ///
    class GazeboYarpIMU : public SensorPlugin
    {
    public:
        GazeboYarpIMU();
        virtual ~GazeboYarpIMU();
        void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

    private:
        yarp::os::Property m_parameters;
        yarp::dev::PolyDriver m_imuDriver;
        yarp::dev::PolyDriver m_MASWrapper;
        yarp::dev::PolyDriver m_AdditionalWrapper; // for the legacy wrapper ServerInertial
        yarp::dev::IMultipleWrapper* m_iWrap{nullptr};
        yarp::dev::IMultipleWrapper* m_iWrapAdditional{nullptr};
        std::string m_scopedSensorName;
        bool m_deviceRegistered;
        std::string m_scopedDeviceName;

    };
}

#endif
