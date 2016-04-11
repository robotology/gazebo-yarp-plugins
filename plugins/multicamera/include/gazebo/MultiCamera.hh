/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#ifndef GAZEBOYARP_MULTICAMERA_HH
#define GAZEBOYARP_MULTICAMERA_HH

#include <gazebo/common/Plugin.hh>
#include <gazebo/plugins/MultiCameraPlugin.hh>
#include <yarp/dev/PolyDriver.h>

#include <yarp/dev/FrameGrabberInterfaces.h>

#include <string>

namespace gazebo
{
    /// \class GazeboYarpMultiCamera
    /// Gazebo Plugin reading from the Gazebo MultiCamera plugin.
    ///
    /// This plugin instantiate a yarp camera driver for the Gazebo simulator
    /// and instantiate a network wrapper (provided by yarp::dev::ServerFrameGrabbers)
    /// to expose the sensor on the yarp network.
    ///
    /// It can be configurated using the yarpConfigurationFile sdf tag,
    /// that contains a Gazebo URI pointing at a yarp .ini configuration file
    ///
    /// The parameter that the yarpConfigurationFile must contain are:
    ///  <TABLE>
    ///  <TR><TD> name </TD><TD> Port name to assign to the wrapper to this device. </TD></TR>
    ///  <TR><TD> period </TD><TD> Update period (in s) of yarp port that publish the measure. </TD></TR>
    ///  </TABLE>
    /// If the required parameters are not specified, their value will be the
    /// default one assigned by the yarp::dev::ServerFrameGrabbers wrapper.
    ///
    class GazeboYarpMultiCamera : public MultiCameraPlugin
    {
    public:
        GazeboYarpMultiCamera();
        virtual ~GazeboYarpMultiCamera();
        virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    private:
        yarp::os::Property m_parameters; 
        yarp::dev::PolyDriver m_cameraDriver;
        std::string m_sensorName;
        sensors::MultiCameraSensor *m_sensor;

        yarp::dev::IFrameGrabberImage*      iFrameGrabberImage;
    };
}



#endif  // GAZEBOYARP_MULTICAMERA_HH
