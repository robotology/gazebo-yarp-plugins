/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#ifndef GAZEBOYARP_DEPTHCAMERA_HH
#define GAZEBOYARP_DEPTHCAMERA_HH

#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/plugins/DepthCameraPlugin.hh>

#include <yarp/os/Network.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/FrameGrabberInterfaces.h>


namespace yarp {
    namespace dev {
        class IMultipleWrapper;
    }
}

namespace gazebo
{
    namespace sensors {
        class DepthCameraSensor;
    }

    /// \class GazeboYarpDepthCamera
    /// Gazebo Plugin reading from the Gazebo DepthCamera plugin, simulating the
    /// robot´s eyes view.
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
    ///  <TR><TD> QUANT_PARAM::depth_quant </TD><TD> The number of decimals for the values of the depth image pixels </TD></TR>
    ///  </TABLE>
    /// If the required parameters are not specified, their value will be the
    /// default one assigned by the yarp::dev::ServerFrameGrabbers wrapper.
    ///
    class GazeboYarpDepthCamera : public DepthCameraPlugin
    {
    public:
        GazeboYarpDepthCamera();
        virtual ~GazeboYarpDepthCamera();
        virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    private:
        yarp::os::Network m_yarp;
        yarp::os::Property m_driverParameters;
        yarp::dev::PolyDriver m_cameraDriver;
        std::string m_sensorName;
        sensors::DepthCameraSensor *m_sensor;

        #ifndef GAZEBO_YARP_PLUGINS_DISABLE_IMPLICIT_NETWORK_WRAPPERS
        yarp::dev::PolyDriver m_cameraWrapper;
        yarp::dev::IMultipleWrapper* m_iWrap;
        #endif

        yarp::dev::IFrameGrabberImage*      iFrameGrabberImage;
    };
}



#endif  // GAZEBOYARP_DEPTHCAMERA_HH
