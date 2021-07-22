/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#ifndef GAZEBOYARP_LASERSENSOR_HH
#define GAZEBOYARP_LASERSENSOR_HH

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
        class LaserSensor;
    }

    /// \class GazeboYarpLaserSensor
    /// Gazebo Plugin emulating the yarp device exposing a 6 axis force-torque sensor.
    ///
    /// This plugin instantiate a yarp 6-axis force torque sensor driver for the Gazebo simulator
    /// and instantiate a network wrapper (provided by yarp::dev::AnalogWrapper)
    /// to expose the sensor on the yarp network.
    ///
    /// It can be configurated using the yarpConfigurationFile sdf tag,
    /// that contains a Gazebo URI pointing at a yarp .ini configuration file
    /// containing the configuration parameters of the controlBoard
    ///
    /// The parameter that the yarpConfigurationFile must contain are:
    ///  <TABLE>
    ///  <TR><TD> robotName </TD><TD> Robot name, used to form the prefix for the opened ports. </TD></TR>
    ///  <TR><TD> deviceId </TD><TD> Id of the device, used to form the prefix for the opened ports. </TD></TR>
    ///  <TR><TD> period </TD><TD> Update period (in ms) of yarp port that publish the measure. It must be an integer </TD></TR>
    ///  </TABLE>
    /// If the required parameters are not specified, their value will be the
    /// default one assigned by the yarp::dev::AnalogWrapper .
    ///
    /// The port opened by the yarp::dev::AnalogWrapper will be:
    ///  <TABLE>
    ///  <TR><TD> /{robotName}/{deviceId}/analog:o  </TD><TD> (port streaming the measure) </TD></TR>
    ///  <TR><TD> /{robotName}/{deviceId}/analog/rpc:i  </TD><TD> (rpc port) </TD></TR>
    ///  </TABLE>
    ///
    class GazeboYarpLaserSensor : public SensorPlugin
    {
    public:
        GazeboYarpLaserSensor();
        virtual ~GazeboYarpLaserSensor();
        
        virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    private:
        #ifndef USE_NEW_WRAPPERS
        yarp::dev::PolyDriver m_laserWrapper;
        yarp::dev::IMultipleWrapper* m_iWrap;
        #endif
        yarp::dev::PolyDriver m_laserDriver;
        
        std::string m_sensorName;
    };
}

#endif
