/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include "CameraDriver.h"

#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>

#include <gazebo/math/Vector3.hh>
#include <gazebo/sensors/CameraSensor.hh>

using namespace std;
using namespace yarp::dev;

const std::string YarpScopedName = "sensorScopedName";

GazeboYarpCameraDriver::GazeboYarpCameraDriver()
{
    m_vertical_flip   = false;
    m_horizontal_flip = false;
}

GazeboYarpCameraDriver::~GazeboYarpCameraDriver()
{
}

//DEVICE DRIVER
bool GazeboYarpCameraDriver::open(yarp::os::Searchable& config)
{
    //Get gazebo pointers
    std::string sensorScopedName(config.find(YarpScopedName.c_str()).asString().c_str());

// TODO get parent sensor, if it make any sense
    m_parentSensor = (gazebo::sensors::CameraSensor*)GazeboYarpPlugins::Handler::getHandler()->getSensor(sensorScopedName);
    if (!m_parentSensor) {
        std::cout << "GazeboYarpCameraDriver Error: camera sensor was not found" << std::endl;
        return false;
    }

    if (config.check("vertical_flip")) m_vertical_flip =true;
    if (config.check("horizonal_flip")) m_horizontal_flip =true;

    m_camera = m_parentSensor->GetCamera();
    if(m_camera == NULL)
    {
        std::cout << "GazeboYarpCameraDriver Error: camera pointer not valid" << std::endl;
        return false;
    }

    m_width  = m_camera->GetImageWidth();
    m_height = m_camera->GetImageHeight();
    m_bufferSize = 3*m_width*m_height;

    m_dataMutex.wait();
    m_imageBuffer = new unsigned char[3*m_width*m_height];
    memset(m_imageBuffer, 0x00, 3*m_width*m_height);
    m_dataMutex.post();

    //Connect the driver to the gazebo simulation
    this->m_updateConnection = m_camera->ConnectNewImageFrame(boost::bind(&GazeboYarpCameraDriver::captureImage, this, _1, _2, _3, _4, _5));
    return true;
}

bool GazeboYarpCameraDriver::close()
{
    if (this->m_updateConnection.get())
    {
        m_camera->DisconnectNewImageFrame(this->m_updateConnection);
        this->m_updateConnection = gazebo::event::ConnectionPtr();
        m_parentSensor = NULL;
    }
    
    delete[] m_imageBuffer;
    m_imageBuffer = 0;

    return true;
}

/*
 * Get the image from the simulator and store it internally
 */
bool GazeboYarpCameraDriver::captureImage(const unsigned char *_image,
                          unsigned int _width, unsigned int _height,
                          unsigned int _depth, const std::string &_format)
{
    m_dataMutex.wait();
    if(m_parentSensor->IsActive())
        memcpy(m_imageBuffer, m_parentSensor->GetImageData(), m_bufferSize);
    m_dataMutex.post();
    return true;
}


// IFRAMEGRABBER IMAGE
bool GazeboYarpCameraDriver::getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& _image)
{
     m_dataMutex.wait();
    _image.resize(m_width, m_height);

    unsigned char *pBuffer = _image.getRawImage();

    if (m_vertical_flip==true && m_horizontal_flip==false)
    {
    int r=0;
    int c=0;
        for (int c=0; c<m_width; c++)
        for (int r=0; r<m_height; r++)
        {
           unsigned char *pixel = _image.getPixelAddress(c,m_height-r-1);
           pixel[0] = *(m_imageBuffer+r*m_width*3+c*3+0);
           pixel[1] = *(m_imageBuffer+r*m_width*3+c*3+1);
           pixel[2] = *(m_imageBuffer+r*m_width*3+c*3+2);
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
           pixel[0] = *(m_imageBuffer+r*m_width*3+c*3+0);
           pixel[1] = *(m_imageBuffer+r*m_width*3+c*3+1);
           pixel[2] = *(m_imageBuffer+r*m_width*3+c*3+2);
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
           pixel[0] = *(m_imageBuffer+r*m_width*3+c*3+0);
           pixel[1] = *(m_imageBuffer+r*m_width*3+c*3+1);
           pixel[2] = *(m_imageBuffer+r*m_width*3+c*3+2);
        }
    }
    else
    {
        memcpy(pBuffer, m_imageBuffer, m_bufferSize);
    }

    m_dataMutex.post();

    return true;
}

int GazeboYarpCameraDriver::height() const
{
    return m_height;
}

int GazeboYarpCameraDriver::width() const
{
    return m_width;
}

//PRECISELY TIMED
yarp::os::Stamp GazeboYarpCameraDriver::getLastInputStamp()
{
    return m_lastTimestamp;
}

int GazeboYarpCameraDriver::getRawBufferSize()
{
    return m_bufferSize;
}

