/*
 * Copyright (C) 2013-2017 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#ifndef GAZEBOYARP_INERTIALMTB_HH
#define GAZEBOYARP_INERTIALMTB_HH

#include <gazebo/common/Plugin.hh>

#include <string>


namespace gazebo
{
    namespace sensors {
        class ImuSensor;
    }

    /// \class GazeboYarpInertialMTB
    /// Gazebo Plugin for the emulation of the yarp inertial MTB device in Gazebo.
    ///
    /// This plugin adds the inertial Gazebo sensor to the Handler database for
    /// later retrieval by the inertial MTB driver. This driver and the associated
    /// network wrapper (provided by yarp::dev::analogServer) will expose the sensor
    /// data on the yarp network.
    ///
    /// The only relevant information needed by this plugin is the sensor scoped name
    /// provided by Gazebo.
    ///
    class GazeboYarpInertialMTB : public SensorPlugin
    {
    public:
        GazeboYarpInertialMTB();
        virtual ~GazeboYarpInertialMTB();
        virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    private:
        std::string m_sensorName;
    };
}

#endif // GAZEBOYARP_INERTIALMTB_HH
