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

using namespace std;
using namespace yarp::dev;

const std::string YarpScopedName = "sensorScopedName";

GazeboYarpDepthCameraDriver::GazeboYarpDepthCameraDriver()
{
    m_vertical_flip     = false;
    m_horizontal_flip   = false;
    m_display_time_box  = false;
    m_display_timestamp = false;

#if GAZEBO_MAJOR_VERSION < 7
    m_imageCameraSensorPtr = 0;
    m_imageCameraPtr       = 0;
#endif
    m_depthCameraSensorPtr = 0;
    m_depthCameraPtr       = 0;

    m_depthFrame_Buffer     = 0;
    m_depthFrame_BufferSize = 0;
    m_imageFrame_Buffer     = 0;
    m_imageFrame_BufferSize = 0;

    // point cloud stuff is not used yet
    m_RGBPointCloud_Buffer = 0;
    m_RGBPointCloud_BufferSize = 0;

    m_updateDepthFrame_Connection = 0;
    m_updateRGBPointCloud_Connection = 0;
    m_updateImageFrame_Connection = 0;

    counter=0;
}


GazeboYarpDepthCameraDriver::~GazeboYarpDepthCameraDriver()
{
}

//DEVICE DRIVER
bool GazeboYarpDepthCameraDriver::open(yarp::os::Searchable &config)
{
    //Get gazebo pointers
    std::string sensorScopedName(config.find(YarpScopedName.c_str()).asString().c_str());

    yTrace() << "GazeboYarpDepthCameraDriver::open() " << sensorScopedName;

#if GAZEBO_MAJOR_VERSION < 7
    m_imageCameraSensorPtr = (gazebo::sensors::CameraSensor*) (GazeboYarpPlugins::Handler::getHandler()->getSensor(sensorScopedName));
    if (!m_imageCameraSensorPtr) {
        yError() << "GazeboYarpDepthCameraDriver Error: camera sensor was not found" ;
        return false;
    }
    m_imageCameraPtr = m_imageCameraSensorPtr->GetCamera();
#endif

    m_depthCameraSensorPtr = dynamic_cast<gazebo::sensors::DepthCameraSensor*> (GazeboYarpPlugins::Handler::getHandler()->getSensor(sensorScopedName));
    if (!m_depthCameraSensorPtr) {
        yError() << "GazeboYarpDepthCameraDriver Error: camera sensor was not found";
        return false;
    }
    #if GAZEBO_MAJOR_VERSION >= 7
        m_depthCameraPtr = this->m_depthCameraSensorPtr->DepthCamera();
    #else
        m_depthCameraPtr = this->m_depthCameraSensorPtr->GetDepthCamera();
    #endif

    // What they are for? Can be removed?
    if (config.check("vertical_flip")) m_vertical_flip =true;
    if (config.check("horizonal_flip")) m_horizontal_flip =true;

#if GAZEBO_MAJOR_VERSION >= 7
    m_width  = m_depthCameraPtr->ImageWidth();
    m_height = m_depthCameraPtr->ImageHeight();
#else
    m_width  = m_imageCameraPtr->GetImageWidth();
    m_height = m_imageCameraPtr->GetImageHeight();
#endif
    yDebug() << "Data from Gazebo:\n width: " << m_width << ", height: " << height();
    m_imageFrame_BufferSize = 3*m_width*m_height;
    m_depthFrame_BufferSize = m_width*m_height*sizeof(float);

    m_colorFrameMutex.wait();
    m_imageFrame_Buffer = new unsigned char[m_imageFrame_BufferSize];
    memset(m_imageFrame_Buffer, 0x00, m_imageFrame_BufferSize);
    m_colorFrameMutex.post();

    m_depthFrameMutex.wait();
    m_depthFrame_Buffer = new float[m_width*m_height];
    memset(m_depthFrame_Buffer, 0x00, m_depthFrame_BufferSize);
    m_depthFrameMutex.post();

//     m_RGBPointCloud_Buffer = new unsigned char[m_RGBPointCloud_BufferSize];
//     memset(m_RGBPointCloud_Buffer, 0x00, m_RGBPointCloud_BufferSize);

    //Connect the driver to the gazebo simulation
    this->m_updateImageFrame_Connection    = m_depthCameraPtr->ConnectNewImageFrame(boost::bind(
                                                                &GazeboYarpDepthCameraDriver::OnNewImageFrame,
                                                                this, _1, _2, _3, _4, _5));

//     this->m_updateRGBPointCloud_Connection = m_depthCameraPtr->ConnectNewRGBPointCloud(boost::bind(
//                                                                 &GazeboYarpDepthCameraDriver::OnNewRGBPointCloud,
//                                                                 this, _1, _2, _3, _4, _5));

    this->m_updateDepthFrame_Connection    = m_depthCameraPtr->ConnectNewDepthFrame(boost::bind(
                                                                &GazeboYarpDepthCameraDriver::OnNewDepthFrame,
                                                                this, _1, _2, _3, _4, _5));

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

#if GAZEBO_MAJOR_VERSION < 7
    m_imageCameraSensorPtr = NULL;
#endif
    m_depthCameraSensorPtr = NULL;

    if(m_depthFrame_Buffer)
        delete[] m_depthFrame_Buffer;
    m_depthFrame_Buffer = 0;
    m_depthFrame_BufferSize = 0;

    if(m_imageFrame_Buffer)
        delete[] m_imageFrame_Buffer;
    m_imageFrame_Buffer = 0;
    m_imageFrame_BufferSize = 0;

    if(m_RGBPointCloud_Buffer)
        delete[] m_RGBPointCloud_Buffer;
    m_RGBPointCloud_Buffer = 0;
    m_RGBPointCloud_BufferSize = 0;

    return true;
}

/*
 * Get the image from the simulator and store it internally
 */
bool GazeboYarpDepthCameraDriver::OnNewImageFrame(const unsigned char *_image,
                          unsigned int _width, unsigned int _height,
                          unsigned int _depth, const std::string &_format)
{

    m_colorFrameMutex.wait();

    #if GAZEBO_MAJOR_VERSION >= 7
    if(m_depthCameraSensorPtr->IsActive())
        memcpy(m_imageFrame_Buffer, m_depthCameraPtr->ImageData(), m_imageFrame_BufferSize);

    m_colorTimestamp.update(this->m_depthCameraSensorPtr->LastUpdateTime().Double());
#else
    if(m_imageCameraSensorPtr->IsActive())
        memcpy(m_imageFrame_Buffer, m_imageCameraSensorPtr->GetImageData(), m_imageFrame_BufferSize);

    m_colorTimestamp.update(this->m_imageCameraSensorPtr->GetLastUpdateTime().Double());
#endif

    m_colorFrameMutex.post();
    return true;
}



/////////////////////////////////////////////////
void GazeboYarpDepthCameraDriver::OnNewRGBPointCloud(const float * /*_pcd*/,
                unsigned int _width, unsigned int _height,
                unsigned int _depth, const std::string &_format)
{
    // Gazebo does not generate pointCouds yet, so nothing to do here!
    return;
}

/////////////////////////////////////////////////
void GazeboYarpDepthCameraDriver::OnNewDepthFrame(const float * /*_image*/,
                                                        unsigned int _width,
                                                        unsigned int _height,
                                                        unsigned int _depth,
                                                        const std::string &_format)
{
    m_depthFrameMutex.wait();
    // for depth camera
    if(m_depthCameraSensorPtr->IsActive())
    #if GAZEBO_MAJOR_VERSION >= 7
        memcpy(m_depthFrame_Buffer, m_depthCameraPtr->DepthData(), m_depthFrame_BufferSize);
    #else
        memcpy(m_depthFrame_Buffer, m_depthCameraPtr->GetDepthData(), m_depthFrame_BufferSize);
    #endif

    m_depthFrameMutex.post();
}

// IFRAMEGRABBER IMAGE
bool GazeboYarpDepthCameraDriver::getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb> &_image)
{
     m_colorFrameMutex.wait();
    _image.resize(m_width, m_height);

    if (m_vertical_flip==true && m_horizontal_flip==false)
    {
        int r=0;
        int c=0;
        for (int c=0; c<m_width; c++)
        for (int r=0; r<m_height; r++)
        {
           unsigned char *pixel = _image.getPixelAddress(c,m_height-r-1);
           pixel[0] = *(m_imageFrame_Buffer + r*m_width*3+c*3+0);
           pixel[1] = *(m_imageFrame_Buffer + r*m_width*3+c*3+1);
           pixel[2] = *(m_imageFrame_Buffer + r*m_width*3+c*3+2);
        }
    }
    else if (m_vertical_flip==false && m_horizontal_flip==true)
    {
        int r=0;
        int c=0;
        for (int c=0; c<m_width; c++)
        for (int r=0; r<m_height; r++)
        {
           unsigned char *pixel = _image.getPixelAddress(m_width-c-1,r);
           pixel[0] = *(m_imageFrame_Buffer + r*m_width*3+c*3+0);
           pixel[1] = *(m_imageFrame_Buffer + r*m_width*3+c*3+1);
           pixel[2] = *(m_imageFrame_Buffer + r*m_width*3+c*3+2);
        }
    }
    else if (m_vertical_flip==true && m_horizontal_flip==true)
    {
        int r=0;
        int c=0;
        for (int c=0; c<m_width; c++)
        for (int r=0; r<m_height; r++)
        {
           unsigned char *pixel = _image.getPixelAddress(m_width-c-1,m_height-r-1);
           pixel[0] = *(m_imageFrame_Buffer + r*m_width*3+c*3+0);
           pixel[1] = *(m_imageFrame_Buffer + r*m_width*3+c*3+1);
           pixel[2] = *(m_imageFrame_Buffer + r*m_width*3+c*3+2);
        }
    }
    else
    {
        memcpy(_image.getRawImage(), m_imageFrame_Buffer, m_imageFrame_BufferSize);
    }

    if (m_display_time_box)
    {
        counter++;
        if (counter == 10) counter = 0; 
        for (int c=0+counter*30; c<30+counter*30; c++)
        for (int r=0; r<30; r++)
        {
           if (counter % 2 ==0)
           {
                unsigned char *pixel = _image.getPixelAddress(m_width-c-1,m_height-r-1);
                pixel[0] = 255;
                pixel[1] = 0;
                pixel[2] = 0;
            }
           else
           {
                unsigned char *pixel = _image.getPixelAddress(m_width-c-1,m_height-r-1);
                pixel[0] = 0;
                pixel[1] = 255;
                pixel[2] = 0;
            }
        }
    }

    m_colorFrameMutex.post();
    return true;
}

bool GazeboYarpDepthCameraDriver::getImage(yarp::sig::ImageOf<yarp::sig::PixelMono> &_image)
{
    return false;
}

int GazeboYarpDepthCameraDriver::height() const
{
    return m_height;
}

int GazeboYarpDepthCameraDriver::width() const
{
    return m_width;
}

int GazeboYarpDepthCameraDriver::getRawBufferSize()
{
    return m_imageFrame_BufferSize;
}

// IRGBDSensor interface
    bool GazeboYarpDepthCameraDriver::getDeviceInfo(yarp::os::Searchable *device_info)
{
    return false;
}

bool GazeboYarpDepthCameraDriver::getMeasurementData(yarp::sig::FlexImage &image, yarp::os::Stamp *stamp)
{
    m_depthFrameMutex.wait();
    memcpy(image.getRawImage(), m_depthFrame_Buffer, m_depthFrame_BufferSize);

    m_depthFrameMutex.post();
    return true;
}

bool GazeboYarpDepthCameraDriver::getDeviceStatus(DepthSensor_status *status)
{
    return false;
}

bool GazeboYarpDepthCameraDriver::getDistanceRange(double *min, double *max)
{
    return false;
}

bool GazeboYarpDepthCameraDriver::setDistanceRange(double min, double max)
{
    return false;
}

bool GazeboYarpDepthCameraDriver::getHorizontalScanLimits(double *min, double *max)
{
    return false;
}

bool GazeboYarpDepthCameraDriver::setHorizontalScanLimits(double min, double max)
{
    return false;
}

bool GazeboYarpDepthCameraDriver::getVerticalScanLimits(double *min, double *max)
{
    return false;
}

bool GazeboYarpDepthCameraDriver::setVerticalScanLimits(double min, double max)
{
    return false;
}

bool GazeboYarpDepthCameraDriver::getDataSize(double *horizontal, double *vertical)
{
    return false;
}

    bool GazeboYarpDepthCameraDriver::setDataSize(double horizontal, double vertical)
{
    return false;
}

    bool GazeboYarpDepthCameraDriver::getResolution(double *hRes, double *vRes)
{
    return false;
}

bool GazeboYarpDepthCameraDriver::setResolution(double hRes, double vRes)
{
    return false;
}

bool GazeboYarpDepthCameraDriver::getScanRate(double *rate)
{
    return false;
}

bool GazeboYarpDepthCameraDriver::setScanRate(double rate)
{
    return false;
}

bool GazeboYarpDepthCameraDriver::getRGBDSensor_Status(RGBDSensor_status *status)
{
    *status = RGBD_SENSOR_OK_IN_USE;
    return true;
}

bool GazeboYarpDepthCameraDriver::getRGBD_Frames(yarp::sig::FlexImage &colorFrame, yarp::sig::FlexImage &depthFrame, yarp::os::Stamp *colorStamp, yarp::os::Stamp *depthStamp)
{
    colorFrame.setPixelCode(VOCAB_PIXEL_RGB);
    colorFrame.setPixelSize(3);
    depthFrame.setPixelCode(VOCAB_PIXEL_MONO_FLOAT);
    depthFrame.setPixelSize(4);
    if( (colorFrame.width() != m_width) || (colorFrame.height() != m_height) )
    {
        colorFrame.resize(m_width, m_height);
    }

    if( (depthFrame.width() != m_width) || (depthFrame.height() != m_height) )
    {
        depthFrame.resize(m_width, m_height);
    }

    m_colorFrameMutex.wait();
    memcpy(colorFrame.getRawImage(), m_imageFrame_Buffer, m_imageFrame_BufferSize);
    m_colorFrameMutex.post();
    m_depthFrameMutex.wait();
    memcpy(depthFrame.getRawImage(), m_depthFrame_Buffer, m_depthFrame_BufferSize);
    m_depthFrameMutex.post();
    if(colorStamp)
        colorStamp->update(m_colorTimestamp.getTime());
    if(depthStamp)
        depthStamp->update(m_depthTimestamp.getTime());

    return true;
}

