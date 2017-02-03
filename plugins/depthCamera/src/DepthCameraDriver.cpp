/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alessandro Settimi and Marco Randazzo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "DepthCameraDriver.h"
#include <yarp/os/Value.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>

#include <gazebo/math/Vector3.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/DepthCameraSensor.hh>
#include <gazebo/rendering/Distortion.hh>
#include <ignition/math/Angle.hh>

#define myError(s) yError() << "GazeboDepthCameraDriver:" << s; m_error = s

using namespace std;
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace ignition::math;

const string YarpScopedName = "sensorScopedName";

GazeboYarpDepthCameraDriver::GazeboYarpDepthCameraDriver()
{
    m_imageFormat                    = VOCAB_PIXEL_RGB;
    m_depthFormat                    = VOCAB_PIXEL_MONO_FLOAT;
    m_width                          = 0;
    m_height                         = 0;
    m_vertical_flip                  = false;
    m_horizontal_flip                = false;
    m_display_time_box               = false;
    m_display_timestamp              = false;
    m_depthCameraSensorPtr           = 0;
    m_depthCameraPtr                 = 0;
    m_depthFrame_Buffer              = 0;
    m_depthFrame_BufferSize          = 0;
    m_imageFrame_Buffer              = 0;
    m_imageFrame_BufferSize          = 0;
    m_updateDepthFrame_Connection    = 0;
    m_updateRGBPointCloud_Connection = 0;
    m_updateImageFrame_Connection    = 0;
    m_counter                        = 0;

    // point cloud stuff is not used yet
    m_RGBPointCloud_Buffer           = 0;
    m_RGBPointCloud_BufferSize       = 0;

    m_format2VocabPixel["L8"]          = VOCAB_PIXEL_MONO;
    m_format2VocabPixel["INT8"]        = VOCAB_PIXEL_MONO;
    m_format2VocabPixel["R8G8B8"]      = VOCAB_PIXEL_RGB;
    m_format2VocabPixel["RGB_INT8"]    = VOCAB_PIXEL_RGB;
    m_format2VocabPixel["BGR_INT8"]    = VOCAB_PIXEL_BGR;
    m_format2VocabPixel["B8G8R8"]      = VOCAB_PIXEL_BGR;
    m_format2VocabPixel["BAYER_RGGB8"] = VOCAB_PIXEL_ENCODING_BAYER_RGGB8;
    m_format2VocabPixel["BAYER_BGGR8"] = VOCAB_PIXEL_ENCODING_BAYER_BGGR8;
    m_format2VocabPixel["BAYER_GBRG8"] = VOCAB_PIXEL_ENCODING_BAYER_GBRG8;
    m_format2VocabPixel["BAYER_GRBG8"] = VOCAB_PIXEL_ENCODING_BAYER_GRBG8;


}

GazeboYarpDepthCameraDriver::~GazeboYarpDepthCameraDriver()
{
}

//DEVICE DRIVER
bool GazeboYarpDepthCameraDriver::open(yarp::os::Searchable &config)
{
    string sensorScopedName((config.find(YarpScopedName.c_str()).asString().c_str()));

    //Get gazebo pointers
    m_conf.fromString(config.toString());
    m_depthCameraSensorPtr = dynamic_cast<gazebo::sensors::DepthCameraSensor*> (GazeboYarpPlugins::Handler::getHandler()->getSensor(sensorScopedName));

    if (!m_depthCameraSensorPtr)
    {
        myError("camera sensor was not found");
        return false;
    }

    m_depthCameraPtr = this->m_depthCameraSensorPtr->DepthCamera();

    m_width  = m_depthCameraPtr->ImageWidth();
    m_height = m_depthCameraPtr->ImageHeight();
    m_imageFrame_BufferSize = m_depthCameraPtr->ImageDepth() * m_width * m_height;
    m_depthFrame_BufferSize = m_width * m_height * sizeof(float);

    m_depthFrameMutex.wait();

    m_depthFrame_Buffer = new float[m_width * m_height];
    memset(m_depthFrame_Buffer, 0x00, m_depthFrame_BufferSize);

    m_depthFrameMutex.post();

    m_colorFrameMutex.wait();

    m_imageFrame_Buffer = new unsigned char[m_imageFrame_BufferSize];
    memset(m_imageFrame_Buffer, 0x00, m_imageFrame_BufferSize);

    m_colorFrameMutex.post();



    //Connect the driver to the gazebo simulation
    auto imageConnectionBind = boost::bind(&GazeboYarpDepthCameraDriver::OnNewImageFrame, this, _1, _2, _3, _4, _5);
    auto depthConnectionBind = boost::bind(&GazeboYarpDepthCameraDriver::OnNewDepthFrame, this, _1, _2, _3, _4, _5);

    this->m_updateImageFrame_Connection = m_depthCameraPtr->ConnectNewImageFrame(imageConnectionBind);
    this->m_updateDepthFrame_Connection = m_depthCameraPtr->ConnectNewDepthFrame(depthConnectionBind);
    return true;
}

bool GazeboYarpDepthCameraDriver::close()
{
    if (this->m_updateImageFrame_Connection.get())
    {
        m_depthCameraPtr->DisconnectNewImageFrame(this->m_updateImageFrame_Connection);
        this->m_updateImageFrame_Connection = gazebo::event::ConnectionPtr();
    }

    if (this->m_updateRGBPointCloud_Connection.get())
    {
        m_depthCameraPtr->DisconnectNewImageFrame(this->m_updateRGBPointCloud_Connection);
        this->m_updateRGBPointCloud_Connection = gazebo::event::ConnectionPtr();
    }

    if (this->m_updateDepthFrame_Connection.get())
    {
        m_depthCameraPtr->DisconnectNewImageFrame(this->m_updateDepthFrame_Connection);
        this->m_updateDepthFrame_Connection = gazebo::event::ConnectionPtr();
    }

    m_depthCameraSensorPtr = NULL;

    if(m_depthFrame_Buffer)
    {
        delete[] m_depthFrame_Buffer;
    }

    m_depthFrame_Buffer     = 0;
    m_depthFrame_BufferSize = 0;

    if(m_imageFrame_Buffer)
    {
        delete[] m_imageFrame_Buffer;
    }

    m_imageFrame_Buffer     = 0;
    m_imageFrame_BufferSize = 0;

    if(m_RGBPointCloud_Buffer)
    {
        delete[] m_RGBPointCloud_Buffer;
    }

    m_RGBPointCloud_Buffer     = 0;
    m_RGBPointCloud_BufferSize = 0;

    return true;
}

/*
 * Get the image from the simulator and store it internally
 */
void GazeboYarpDepthCameraDriver::OnNewImageFrame(const unsigned char* _image, UInt _width, UInt _height, UInt _depth, const string& _format)
{
    //possible image format (hardcoded (sigh..) in osrf/gazebo/source/gazebo/rendering/Camera.cc )
    /* data type is string. why they didn't use a enum? mystery...
     * 
     * L8 = INT8 = 1
     * R8G8B8 = RGB_INT8 = BGR_INT8 = B8G8R8 = 3
     * BAYER_RGGB8 = BAYER_BGGR8 = BAYER_GBRG8 = BAYER_GRBG8 = 1
     * */
    m_colorFrameMutex.wait();

    if(m_format2VocabPixel.find(_format) == m_format2VocabPixel.end())
    {
        myError("format not supported");
        return;
    }

    m_imageFormat = m_format2VocabPixel[_format];
    m_width       = _width;
    m_height      = _height;

    if(m_imageFrame_BufferSize != _width * _height * _depth)
    {
        m_imageFrame_BufferSize = _width * _height * _depth;
        m_imageFrame_Buffer     = new unsigned char[_width * _height * _depth];
    }

    if(m_depthCameraSensorPtr->IsActive())
    {
        memcpy(m_imageFrame_Buffer, _image, _width * _height * _depth);
    }

    m_colorTimestamp.update(this->m_depthCameraSensorPtr->LastUpdateTime().Double());
    m_colorFrameMutex.post();
    return;
}

void GazeboYarpDepthCameraDriver::OnNewRGBPointCloud(const float * /*_pcd*/, UInt _width, UInt _height, UInt _depth, const string &_format)
{
    // Gazebo does not generate pointCouds yet, so nothing to do here!
    return;
}

void GazeboYarpDepthCameraDriver::OnNewDepthFrame(const float* image, UInt _width, UInt _height, UInt _depth, const string& _format)
{
    m_depthFrameMutex.wait();

    if (_format != "FLOAT32")
    {
        myError("image format not recognized!");
    }

    m_depthFormat = VOCAB_PIXEL_MONO_FLOAT;
    m_width       = _width;
    m_height      = _height;

    if(m_depthFrame_BufferSize != _width * _height * sizeof(float))
    {
        m_depthFrame_BufferSize = _width * _height * sizeof(float);
        m_depthFrame_Buffer     = new float[_width * _height];
    }

    if(m_depthCameraSensorPtr->IsActive())
    {
        memcpy(m_depthFrame_Buffer, image, _width * _height * sizeof(float));
        m_depthTimestamp.update(this->m_depthCameraSensorPtr->LastUpdateTime().Double());
    }
    
    m_depthFrameMutex.post();
}

//IRGBDSensor
int GazeboYarpDepthCameraDriver::getRgbHeight()
{
    return m_height;
}

int GazeboYarpDepthCameraDriver::getRgbWidth()
{
    return m_width;
}
bool GazeboYarpDepthCameraDriver::setRgbResolution(int width, int height)
{
    m_depthCameraSensorPtr->DepthCamera()->SetImageHeight(height);
    m_depthCameraSensorPtr->DepthCamera()->SetImageWidth(width);

    m_width  = width;
    m_height = height;
    return true;
}

bool GazeboYarpDepthCameraDriver::getRgbFOV(double& horizontalFov, double& verticalFov)
{
    horizontalFov = m_depthCameraSensorPtr->DepthCamera()->HFOV().Degree();
    verticalFov   = m_depthCameraSensorPtr->DepthCamera()->VFOV().Degree();
    return true;
}
bool GazeboYarpDepthCameraDriver::setRgbFOV(double horizontalFov, double verticalFov)
{
    ignition::math::Angle hFov;
    hFov.Degree(horizontalFov);
    m_depthCameraSensorPtr->DepthCamera()->SetHFOV(hFov);
    yWarning() << "GazeboDepthCameraDriver: only horizontal fov setted!";
    return true;
}
bool GazeboYarpDepthCameraDriver::getRgbMirroring(bool& mirror)
{
    mirror = false;
    return true;
}
bool GazeboYarpDepthCameraDriver::setRgbMirroring(bool mirror)
{
    myError("not implemented yet");
    return false;
}
bool GazeboYarpDepthCameraDriver::getRgbIntrinsicParam(Property& intrinsic)
{
    return getDepthIntrinsicParam(intrinsic);
}
bool GazeboYarpDepthCameraDriver::getRgbImage(FlexImage& rgbImage, Stamp* timeStamp)
{
    if(!timeStamp)
    {
        myError("timestamp pointer invalid!");
        return false;
    }

    m_colorFrameMutex.wait();

    if(m_width == 0 || m_height == 0)
    {
        myError("gazebo returned an invalid image size");
        return false;
    }
    rgbImage.setPixelCode(m_imageFormat);
    rgbImage.resize(m_width, m_height);
    memcpy(rgbImage.getRawImage(), m_imageFrame_Buffer, m_imageFrame_BufferSize);
    timeStamp->getTime();

    m_colorFrameMutex.post();
    return true;
}

int GazeboYarpDepthCameraDriver::getDepthHeight()
{
    return m_height;
}
int GazeboYarpDepthCameraDriver::getDepthWidth()
{
    return m_width;
}
bool GazeboYarpDepthCameraDriver::setDepthResolution(int width, int height)
{
    return setRgbResolution(width, height);
}
bool GazeboYarpDepthCameraDriver::getDepthFOV(double& horizontalFov, double& verticalFov)
{
    return getRgbFOV(horizontalFov, verticalFov);
}
bool GazeboYarpDepthCameraDriver::setDepthFOV(double horizontalFov, double verticalFov)
{
    return getRgbFOV(horizontalFov, verticalFov);
}
bool GazeboYarpDepthCameraDriver::getDepthIntrinsicParam(Property& intrinsic)
{
    using namespace gazebo::rendering;

    Distortion*  distModel;
    DepthCamera* camPtr;
    Value        retM;

    camPtr = m_depthCameraSensorPtr->DepthCamera().get();

    if(camPtr)
    {
        distModel = camPtr->LensDistortion().get();
        if(distModel)
        {
            intrinsic.put("focalLengthX",    camPtr->OgreCamera()->getFocalLength());
            intrinsic.put("focalLengthY",    camPtr->OgreCamera()->getFocalLength() * camPtr->OgreCamera()->getAspectRatio());
            intrinsic.put("k1",              distModel->GetK1());
            intrinsic.put("k2",              distModel->GetK2());
            intrinsic.put("k3",              distModel->GetK3());
            intrinsic.put("t1",              distModel->GetP1());
            intrinsic.put("t2",              distModel->GetP2());
            intrinsic.put("principalPointX", distModel->GetCenter().x);
            intrinsic.put("principalPointY", distModel->GetCenter().y);
        }
    }
    intrinsic.put("retificationMatrix", retM.makeList("1.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 1.0"));
    intrinsic.put("distortionModel", "plumb_bob");
    intrinsic.put("stamp", m_colorTimestamp.getTime());
    return true;
}

double GazeboYarpDepthCameraDriver::getDepthAccuracy()
{
    return 0.00001;
}

bool GazeboYarpDepthCameraDriver::setDepthAccuracy(double accuracy)
{
    myError("impossible to set accuracy");
    return false;
}

bool GazeboYarpDepthCameraDriver::getDepthClipPlanes(double& nearPlane, double& farPlane)
{
    nearPlane = m_depthCameraSensorPtr->DepthCamera()->NearClip();
    farPlane  = m_depthCameraSensorPtr->DepthCamera()->FarClip();
}
bool GazeboYarpDepthCameraDriver::setDepthClipPlanes(double nearPlane, double farPlane)
{
    m_depthCameraSensorPtr->DepthCamera()->SetClipDist(nearPlane,farPlane);
    return true;
}
bool GazeboYarpDepthCameraDriver::getDepthMirroring(bool& mirror)
{
    return getRgbMirroring(mirror);
}
bool GazeboYarpDepthCameraDriver::setDepthMirroring(bool mirror)
{
    return setRgbMirroring(mirror);
}
bool GazeboYarpDepthCameraDriver::getDepthImage(depthImageType& depthImage, Stamp* timeStamp)
{
    if(!timeStamp)
    {
        myError("gazeboDepthCameraDriver: timestamp pointer invalid!");
        return false;
    }
    m_depthFrameMutex.wait();

    if(m_width == 0 || m_height == 0)
    {
        myError("gazebo returned an invalid image size");
        return false;
    }

    depthImage.resize(m_width, m_height);
    //depthImage.setPixelCode(m_depthFormat);
    memcpy(depthImage.getRawImage(), m_depthFrame_Buffer, m_width * m_height * sizeof(float));
    timeStamp->getTime();

    m_depthFrameMutex.post();
    return true;
}
bool GazeboYarpDepthCameraDriver::getExtrinsicParam(sig::Matrix& extrinsic)
{
    extrinsic.resize(4, 4);
    extrinsic.zero();
    extrinsic[1][1] = extrinsic[2][2] = extrinsic[3][3] = extrinsic[4][4] = 1;
    return true;
}
bool GazeboYarpDepthCameraDriver::getImages(FlexImage& colorFrame, depthImageType& depthFrame, Stamp* colorStamp, Stamp* depthStamp)
{
    return getDepthImage(depthFrame, depthStamp) && getRgbImage(colorFrame, colorStamp);
}

IRGBDSensor::RGBDSensor_status GazeboYarpDepthCameraDriver::getSensorStatus()
{
    return m_depthCameraSensorPtr->IsActive() ? RGBD_SENSOR_OK_IN_USE : RGBD_SENSOR_NOT_READY;
}
yarp::os::ConstString GazeboYarpDepthCameraDriver::getLastErrorMsg(Stamp* timeStamp)
{
    if(!timeStamp)
    {
        myError("timeStamp pointer invalid");
    }
    else
    {
        timeStamp->update();
    }
    return m_error;
}
