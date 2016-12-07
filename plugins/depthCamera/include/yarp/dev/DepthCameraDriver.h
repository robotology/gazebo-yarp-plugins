/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alessandro Settimi and Marco Randazzo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_DEPTHCAMERADRIVER_H
#define GAZEBOYARP_DEPTHCAMERADRIVER_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>

#include <boost/shared_ptr.hpp>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/DepthCameraSensor.hh>


//Forward declarations
namespace yarp {
    namespace dev {
        class GazeboYarpDepthCameraDriver;
    }
}

namespace gazebo {
    namespace common {
        class UpdateInfo;
    }
    namespace sensors {
        class DepthCameraSensor;
    }
    namespace event {
        class Connection;
        typedef boost::shared_ptr<Connection> ConnectionPtr;
    }
}

extern const std::string YarpScopedName;

class yarp::dev::GazeboYarpDepthCameraDriver:   public yarp::dev::DeviceDriver,
                                                public yarp::dev::IRGBDSensor
{
    typedef unsigned int                              UInt;
    typedef yarp::sig::ImageOf<yarp::sig::PixelFloat> depthImageType;
    typedef yarp::os::Stamp                           Stamp;
    typedef yarp::os::Property                        Property;
    typedef yarp::sig::FlexImage                      FlexImage;
    typedef std::string                               string;
public:
    GazeboYarpDepthCameraDriver();

    virtual ~GazeboYarpDepthCameraDriver();

    //DEVICE DRIVER
    virtual bool   open(yarp::os::Searchable& config);
    virtual bool   close();

    //IRGBDSensor
    virtual int                   getRgbHeight();
    virtual int                   getRgbWidth();
    virtual bool                  setRgbResolution(int width, int height);
    virtual bool                  getRgbFOV(double& horizontalFov, double& verticalFov);
    virtual bool                  setRgbFOV(double horizontalFov, double verticalFov);
    virtual bool                  getRgbMirroring(bool& mirror);
    virtual bool                  setRgbMirroring(bool mirror);
    virtual bool                  getRgbIntrinsicParam(Property& intrinsic);
    virtual bool                  getRgbImage(FlexImage& rgbImage, Stamp* timeStamp = NULL);

    virtual int                   getDepthHeight();
    virtual int                   getDepthWidth();
    virtual bool                  setDepthResolution(int width, int height);
    virtual bool                  getDepthFOV(double& horizontalFov, double& verticalFov);
    virtual bool                  setDepthFOV(double horizontalFov, double verticalFov);
    virtual bool                  getDepthIntrinsicParam(Property& intrinsic);
    virtual double                getDepthAccuracy();
    virtual bool                  setDepthAccuracy(double accuracy);
    virtual bool                  getDepthClipPlanes(double& nearPlane, double& farPlane);
    virtual bool                  setDepthClipPlanes(double nearPlane, double farPlane);
    virtual bool                  getDepthMirroring(bool& mirror);
    virtual bool                  setDepthMirroring(bool mirror);
    virtual bool                  getDepthImage(depthImageType& depthImage, Stamp* timeStamp = NULL);

    virtual bool                  getExtrinsicParam(sig::Matrix &extrinsic);
    virtual bool                  getImages(FlexImage& colorFrame, depthImageType& depthFrame, Stamp* colorStamp=NULL, Stamp* depthStamp=NULL);
    virtual RGBDSensor_status     getSensorStatus();
    virtual yarp::os::ConstString getLastErrorMsg(Stamp* timeStamp = NULL);
    /*
     * INTERFACE TOWARD GAZEBO SIMULATOR CORE
     * Get the image from the simulator and store it internally
     */
    void OnNewImageFrame(const unsigned char *_image, UInt _width, UInt _height, UInt _depth, const std::string &_format);
    void OnNewRGBPointCloud(const float * /*_pcd*/, UInt /*_width*/, UInt /*_height*/, UInt /*_depth*/, const std::string &/*_format*/);
    void OnNewDepthFrame(const float * /*_image*/, UInt /*_width*/, UInt /*_height*/, UInt /*_depth*/, const std::string &/*_format*/);

private:
    yarp::os::Property  m_conf;

    int                 m_imageFrame_BufferSize;
    int                 m_depthFrame_BufferSize;
    int                 m_RGBPointCloud_BufferSize;

    UInt                m_width;
    UInt                m_height;
    bool                m_vertical_flip;
    bool                m_horizontal_flip;
    bool                m_display_time_box;
    bool                m_display_timestamp;

    yarp::os::Stamp     m_colorTimestamp; // last timestamp data
    yarp::os::Stamp     m_depthTimestamp; // last timestamp data
    yarp::os::Semaphore m_colorFrameMutex;  //mutex for accessing the data
    yarp::os::Semaphore m_depthFrameMutex;  //mutex for accessing the data

    unsigned char*      m_imageFrame_Buffer;
    float*              m_depthFrame_Buffer;
    unsigned char*      m_RGBPointCloud_Buffer;
    int                 m_counter;
    string              m_error;

    std::map<string, YarpVocabPixelTypesEnum> m_format2VocabPixel;
    YarpVocabPixelTypesEnum                   m_imageFormat;
    YarpVocabPixelTypesEnum                   m_depthFormat;
    gazebo::sensors::DepthCameraSensor*       m_depthCameraSensorPtr;
    gazebo::rendering::DepthCameraPtr         m_depthCameraPtr;
    gazebo::event::ConnectionPtr              m_updateDepthFrame_Connection;
    gazebo::event::ConnectionPtr              m_updateRGBPointCloud_Connection;
    gazebo::event::ConnectionPtr              m_updateImageFrame_Connection;
};

#endif // GAZEBOYARP_DEPTHCAMERADRIVER_H
