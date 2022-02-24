/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alessandro Settimi and Marco Randazzo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "DepthCameraDriver.h"
#include "DepthCameraLog.h"
#include <yarp/os/Value.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>

#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/DepthCameraSensor.hh>
#include <gazebo/rendering/Distortion.hh>
#include <ignition/math/Angle.hh>

using namespace std;
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace ignition::math;
using GazeboYarpPlugins::GAZEBODEPTH;

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

    //Manage depth quantization parameter
    if(config.check("QUANT_PARAM")) {
        yarp::os::Property quantCfg;
        quantCfg.fromString(config.findGroup("QUANT_PARAM").toString());
        m_depthQuantizationEnabled = true;
        if (quantCfg.check("depth_quant")) {
            m_depthDecimalNum = quantCfg.find("depth_quant").asInt32();
        }
    }

    //Get gazebo pointers
    m_conf.fromString(config.toString());
    m_depthCameraSensorPtr = dynamic_cast<gazebo::sensors::DepthCameraSensor*> (GazeboYarpPlugins::Handler::getHandler()->getSensor(sensorScopedName));

    if (!m_depthCameraSensorPtr)
    {
        yCError(GAZEBODEPTH, "camera sensor was not found (sensor's scoped name %s!)", sensorScopedName.c_str());
        return false;
    }

    m_depthCameraPtr = this->m_depthCameraSensorPtr->DepthCamera();

    m_width  = m_depthCameraPtr->ImageWidth();
    m_height = m_depthCameraPtr->ImageHeight();
    m_imageFrame_BufferSize = m_depthCameraPtr->ImageDepth() * m_width * m_height;
    m_depthFrame_BufferSize = m_width * m_height * sizeof(float);


    {
        std::lock_guard<std::mutex> lock(m_depthFrameMutex);
        m_depthFrame_Buffer = new float[m_width * m_height];
        memset(m_depthFrame_Buffer, 0x00, m_depthFrame_BufferSize);
    }

    {
        std::lock_guard<std::mutex> lock(m_colorFrameMutex);
        m_imageFrame_Buffer = new unsigned char[m_imageFrame_BufferSize];
        memset(m_imageFrame_Buffer, 0x00, m_imageFrame_BufferSize);
    }

    //Connect the driver to the gazebo simulation
    using namespace boost::placeholders;
    auto imageConnectionBind = boost::bind(&GazeboYarpDepthCameraDriver::OnNewImageFrame, this, _1, _2, _3, _4, _5);
    auto depthConnectionBind = boost::bind(&GazeboYarpDepthCameraDriver::OnNewDepthFrame, this, _1, _2, _3, _4, _5);

    this->m_updateImageFrame_Connection = m_depthCameraPtr->ConnectNewImageFrame(imageConnectionBind);
    this->m_updateDepthFrame_Connection = m_depthCameraPtr->ConnectNewDepthFrame(depthConnectionBind);

    return true;
}

bool GazeboYarpDepthCameraDriver::close()
{
    this->m_updateImageFrame_Connection.reset();
    this->m_updateRGBPointCloud_Connection.reset();
    this->m_updateDepthFrame_Connection.reset();

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

    std::lock_guard<std::mutex> lock(m_colorFrameMutex);

    if(m_format2VocabPixel.find(_format) == m_format2VocabPixel.end())
    {
        yCError(GAZEBODEPTH) << "format not supported";
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
    return;
}

void GazeboYarpDepthCameraDriver::OnNewRGBPointCloud(const float * /*_pcd*/, UInt _width, UInt _height, UInt _depth, const string &_format)
{
    // Gazebo does not generate pointCouds yet, so nothing to do here!
    return;
}

void GazeboYarpDepthCameraDriver::OnNewDepthFrame(const float* image, UInt _width, UInt _height, UInt _depth, const string& _format)
{
    std::lock_guard<std::mutex> lock(m_depthFrameMutex);

    if (_format != "FLOAT32")
    {
        yCError(GAZEBODEPTH) << "image format not recognized!";
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
    yCWarning(GAZEBODEPTH) << "GazeboDepthCameraDriver: only horizontal fov set!";
    return true;
}
bool GazeboYarpDepthCameraDriver::getRgbMirroring(bool& mirror)
{
    mirror = false;
    return true;
}
bool GazeboYarpDepthCameraDriver::setRgbMirroring(bool mirror)
{
    yCError(GAZEBODEPTH)  << "setRgbMirroring not implemented yet";
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
        yCError(GAZEBODEPTH) << "timestamp pointer invalid!";
        return false;
    }

    std::lock_guard<std::mutex> lock(m_colorFrameMutex);

    if(m_width == 0 || m_height == 0)
    {
        yCError(GAZEBODEPTH)  << "gazebo returned an invalid image size";
        return false;
    }
    rgbImage.setPixelCode(m_imageFormat);
    rgbImage.resize(m_width, m_height);
    memcpy(rgbImage.getRawImage(), m_imageFrame_Buffer, m_imageFrame_BufferSize);
    timeStamp->update(this->m_depthCameraSensorPtr->LastUpdateTime().Double());

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
    Value        rectM;

    intrinsic.put("physFocalLength", 0.0);
    camPtr = m_depthCameraSensorPtr->DepthCamera().get();
    if(camPtr)
    {
        intrinsic.put("focalLengthX",    1. / camPtr->OgreCamera()->getPixelDisplayRatio());
        intrinsic.put("focalLengthY",    1. / camPtr->OgreCamera()->getPixelDisplayRatio());
        distModel = camPtr->LensDistortion().get();
        if(distModel)
        {
            intrinsic.put("k1",              distModel->K1());
            intrinsic.put("k2",              distModel->K2());
            intrinsic.put("k3",              distModel->K3());
            intrinsic.put("t1",              distModel->P1());
            intrinsic.put("t2",              distModel->P2());
            intrinsic.put("principalPointX", distModel->Center().X());
            intrinsic.put("principalPointY", distModel->Center().Y());
        }
        else
        {
            intrinsic.put("k1",              0.0);
            intrinsic.put("k2",              0.0);
            intrinsic.put("k3",              0.0);
            intrinsic.put("t1",              0.0);
            intrinsic.put("t2",              0.0);
            intrinsic.put("principalPointX", m_width/2.0);
            intrinsic.put("principalPointY", m_height/2.0);
        }

    }
    intrinsic.put("rectificationMatrix", rectM.makeList("1.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 1.0"));
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
    yCError(GAZEBODEPTH)  << "impossible to set accuracy";
    return false;
}

bool GazeboYarpDepthCameraDriver::getDepthClipPlanes(double& nearPlane, double& farPlane)
{
    nearPlane = m_depthCameraSensorPtr->DepthCamera()->NearClip();
    farPlane  = m_depthCameraSensorPtr->DepthCamera()->FarClip();
    return true;
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
        yCError(GAZEBODEPTH)  << "gazeboDepthCameraDriver: timestamp pointer invalid!";
        return false;
    }

    std::lock_guard<std::mutex> lock(m_depthFrameMutex);

    if(m_width == 0 || m_height == 0)
    {
        yCError(GAZEBODEPTH)  << "gazebo returned an invalid image size";
        return false;
    }

    depthImage.resize(m_width, m_height);
    //depthImage.setPixelCode(m_depthFormat);
    if(!m_depthQuantizationEnabled) {
        memcpy(depthImage.getRawImage(), m_depthFrame_Buffer, m_width * m_height * sizeof(float));
    }
    else {
        double nearPlane = m_depthCameraSensorPtr->DepthCamera()->NearClip();
        double farPlane = m_depthCameraSensorPtr->DepthCamera()->FarClip();

        int intTemp;
        float value;

        auto pxPtr = reinterpret_cast<float*>(depthImage.getRawImage());
        for(int i=0; i<m_height*m_width; i++){
            value = m_depthFrame_Buffer[i];

            intTemp = (int) (value * pow(10.0, (float) m_depthDecimalNum));
            value = (float) intTemp / pow(10.0, (float) m_depthDecimalNum);

            if (value < nearPlane) { value = nearPlane; }
            if (value > farPlane) { value = farPlane; }

            *pxPtr = value;
            pxPtr++;
        }
    }

    timeStamp->update(this->m_depthCameraSensorPtr->LastUpdateTime().Double());

    return true;
}
bool GazeboYarpDepthCameraDriver::getExtrinsicParam(yarp::sig::Matrix& extrinsic)
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
std::string GazeboYarpDepthCameraDriver::getLastErrorMsg(Stamp* timeStamp)
{
    if(timeStamp)
    {
        timeStamp->update(this->m_depthCameraSensorPtr->LastUpdateTime().Double());

    }
    return m_error;
}
