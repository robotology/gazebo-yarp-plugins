/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include "CameraDriver.h"
#include "Handler.hh"
#include "common.h"

#include <gazebo/math/Vector3.hh>
#include <gazebo/sensors/CameraSensor.hh>

using namespace std;
using namespace yarp::dev;

const std::string YarpScopedName = "sensorScopedName";

GazeboYarpCameraDriver::GazeboYarpCameraDriver() {}
GazeboYarpCameraDriver::~GazeboYarpCameraDriver() {}
    
//DEVICE DRIVER
bool GazeboYarpCameraDriver::open(yarp::os::Searchable& config)
{
    //Get gazebo pointers
    std::string sensorScopedName(config.find(YarpScopedName.c_str()).asString().c_str());
//    std::cout << "GazeboYarpCameraDriver is looking for sensor " << sensorScopedName << std::endl;
//    std::cout << "GazeboYarpCameraDriver open parameters are " << config.toString() << std::endl;

// TODO get parent sensor, if it make any sense
    m_parentSensor = (gazebo::sensors::CameraSensor*)GazeboYarpPlugins::Handler::getHandler()->getSensor(sensorScopedName);
    if (!m_parentSensor) {
        std::cout << "GazeboYarpCameraDriver Error: camera sensor was not found" << std::endl;
        return false;
    }

    _camera = m_parentSensor->GetCamera();
    if(_camera == NULL)
    {
        std::cout << "GazeboYarpCameraDriver Error: camera pointer not valid" << std::endl;
        return false;
    }
    std::cout << " image size is: " << this->m_parentSensor->GetImageWidth() << "x" << this->m_parentSensor->GetImageHeight() << std::endl;

    _camera->EnableSaveFrame(true);

    _width  = 320;
    _height = 240;
    _bufferSize = 3*_width*_height;

    m_dataMutex.wait();
    imageBuffer = new unsigned char[3*_width*_height];
    memset(imageBuffer, 0x00, 3*_width*_height);
    m_dataMutex.post();

    //Connect the driver to the gazebo simulation
    this->m_updateConnection = _camera->ConnectNewImageFrame(boost::bind(&GazeboYarpCameraDriver::captureImage, this, _1, _2, _3, _4, _5));
    return true;
}

bool GazeboYarpCameraDriver::close()
{
    if (this->m_updateConnection.get())
    {
        _camera->DisconnectNewImageFrame(this->m_updateConnection);
        this->m_updateConnection = gazebo::event::ConnectionPtr();
        m_parentSensor = NULL;
    }
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
        memcpy(imageBuffer, m_parentSensor->GetImageData(), _bufferSize);
    m_dataMutex.post();
    return true;
}


// IFRAMEGRABBER IMAGE
bool GazeboYarpCameraDriver::getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& _image)
{
    static int alive = 0;

     m_dataMutex.wait();
    _image.resize(_width, _height);

    unsigned char *pBuffer = _image.getRawImage();


    memcpy(pBuffer, imageBuffer, _bufferSize);
    m_dataMutex.post();

    alive++;
    if(alive >= 255)
    {
        std::cout << "GazeboYarpCamera getImage" << endl;
        alive = 0;
    }
    return true;
}

int GazeboYarpCameraDriver::height() const
{
    std::cout << "GazeboYarpCamera height" << endl;
    return 0;
}

int GazeboYarpCameraDriver::width() const
{
    std::cout << "GazeboYarpCamera width" << endl;
    return 0;
}

//PRECISELY TIMED
yarp::os::Stamp GazeboYarpCameraDriver::getLastInputStamp()
{
    return m_lastTimestamp;
}

int GazeboYarpCameraDriver::getRawBufferSize()
{
    return _bufferSize;
}

