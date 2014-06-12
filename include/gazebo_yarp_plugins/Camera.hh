/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#ifndef GAZEBOYARP_CAMERA_HH
#define GAZEBOYARP_CAMERA_HH

#include <gazebo/common/Plugin.hh>
#include <gazebo/plugins/CameraPlugin.hh>
//#include <gazebo_yarp_plugins/CameraDriver.h>
#include <yarp/os/Network.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/dev/FrameGrabberInterfaces.h>

#include <string>


class IRobotranYarpInterface;


class YARP_dev_API IRobotranYarpInterface
{
    virtual bool captureImage(const unsigned char *_image,
                              unsigned int _width, unsigned int _height,
                              unsigned int _depth, const std::string &_format) = 0;
};

namespace gazebo
{
    namespace sensors {
        class CameraSensor;
    }

    /// \class GazeboYarpCamera
    /// Gazebo Plugin reading from the Gazebo Camera plugin, simulating the
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
    ///  </TABLE>
    /// If the required parameters are not specified, their value will be the
    /// default one assigned by the yarp::dev::ServerFrameGrabbers wrapper.
    ///
    class GazeboYarpCamera : public CameraPlugin
    {
    public:
        GazeboYarpCamera();
        virtual ~GazeboYarpCamera();
        virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

//        //CAMERA UPDATE from Gazebo side
//        virtual void OnNewFrame(const unsigned char *_image,
//                              unsigned int _width, unsigned int _height,
//                              unsigned int _depth, const std::string &_format);

    private:
        yarp::os::Network m_yarp;
        yarp::os::Property m_parameters; 
        yarp::dev::PolyDriver m_cameraDriver;
        yarp::dev::PolyDriver _cameraDriver;
        std::string m_sensorName;
        sensors::CameraSensor *m_sensor;

        yarp::dev::IFrameWriterImage*       iFrameWriterImage;
        yarp::dev::IFrameGrabber*           iFrameGrabber;
        yarp::dev::IFrameGrabberImage*      iFrameGrabberImage;
        yarp::dev::DeviceDriver *           dev;
    };
}



#endif  // GAZEBOYARP_CAMERA_HH
