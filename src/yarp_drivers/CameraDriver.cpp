/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include "gazebo_yarp_plugins/CameraDriver.h"

#include "gazebo_yarp_plugins/Handler.hh"
#include "gazebo_yarp_plugins/common.h"

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
    std::cout << "GazeboYarpCameraDriver is looking for sensor " << sensorScopedName << std::endl;
    std::cout << "GazeboYarpCameraDriver open parameters are " << config.toString() << std::endl;

    std::cout << "my address is " << this << std::endl;
    std::cout << "dynamic cast of this (IRobotranYarpInterface) is " << dynamic_cast<IRobotranYarpInterface*> (this) << std::endl;
    std::cout << "dynamic cast of this (IFrameGrabberImage) is" << dynamic_cast<yarp::dev::IFrameGrabberImage*> (this) << std::endl;

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
    std::cout << " image width is: " << this->m_parentSensor->GetImageWidth() << " and height is " << this->m_parentSensor->GetImageHeight();

    _camera->EnableSaveFrame(true);

    _width  = 320;
    _height = 240;
    _bufferSize = 3*_width*_height;

    m_dataMutex.wait();
    imageBuffer = new unsigned char[3*_width*_height];
    memset(imageBuffer, 0x00, 3*_width*_height);
    m_dataMutex.post();

    //Connect the driver to the gazebo simulation
//    this->m_updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboYarpCameraDriver::onUpdate, this, _1));
  
    this->m_updateConnection = _camera->ConnectNewImageFrame(boost::bind(&GazeboYarpCameraDriver::captureImage, this, _1, _2, _3, _4, _5));
    return true;
}

bool GazeboYarpCameraDriver::close()
{
    if (this->m_updateConnection.get()) {
        gazebo::event::Events::DisconnectWorldUpdateBegin(this->m_updateConnection);
        this->m_updateConnection = gazebo::event::ConnectionPtr();
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
    std::cout << " MY lovely device GazeboYarpCameraDriver::captureImage" << std::endl;
    m_dataMutex.wait();
    memcpy(imageBuffer, m_parentSensor->GetImageData(), _bufferSize);
    m_dataMutex.post();
    return true;
}

///**
// * Export a camera sensor.
// *
// * Copy a frame from Gazebo to internal memory
// */
void GazeboYarpCameraDriver::OnNewFrame(const unsigned char *_image,
                      unsigned int _width, unsigned int _height,
                      unsigned int _depth, const std::string &_format)
{
    std::cout << " MY lovely device GazeboYarpCameraDriver::OnNewFrame\n" << std::endl;

//        static int saveCount = 0;

//        char tmp[1024];
//        snprintf(tmp, sizeof(tmp), "/tmp/gazeboCameras/%s-%04d.jpg", _camera->GetName().c_str(), saveCount);

//        if (saveCount < 1000)
//        {
//            this->_camera->SaveFrame( _image, _width, _height, _depth, _format, tmp);

//            std::cout << "Saving frame [" << saveCount << "] as [" << tmp << "]\n" << std::endl;
//            saveCount++;
//        }

    return;
}
    

void GazeboYarpCameraDriver::onUpdate(const gazebo::common::UpdateInfo& info)
{
    static int onUpdate = 0;

    if(m_parentSensor->IsActive())
    {
        size_t imageSize =_camera->GetImageByteSize();
        if(imageSize != _bufferSize)
        {
            std::cout << "imageSize from Gazebo (" << imageSize << ") differs from expected (" << _bufferSize << ")" << std::endl;
            return;
        }
        m_dataMutex.wait();
        memcpy(imageBuffer, m_parentSensor->GetImageData(), _bufferSize);


//        char tmp[1024];
//        snprintf(tmp, sizeof(tmp), "/tmp/gazeboCameras/%s-%04d.jpg", this->m_parentSensor->GetCamera()->GetName().c_str(), onUpdate);

//        if (saveCount < 100)
//        {
//            this->m_parentSensor->GetCamera()->SaveFrame( imageBuffer, _width, _height, _depth, _format, tmp);

//            std::cout << "Saving frame [" << onUpdate << "] as [" << tmp << "]\n" << std::endl;
//            saveCount++;
//        }
        m_dataMutex.post();



        onUpdate++;
        if(onUpdate >= 2505)
        {
            std::cout << "GazeboYarpCamera onUpdate" << std::endl;
            onUpdate = 0;
        }
    }
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

bool GazeboYarpCameraDriver::putImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image)
{
    std::cout << "GazeboYarpCamera putImage" << endl;
    return true;
}


//PRECISELY TIMED
yarp::os::Stamp GazeboYarpCameraDriver::getLastInputStamp()
{
    return m_lastTimestamp;
}


bool GazeboYarpCameraDriver::getRawBuffer(unsigned char *buffer)
{
    return true;
}

int GazeboYarpCameraDriver::getRawBufferSize()
{
    return 0;
}

