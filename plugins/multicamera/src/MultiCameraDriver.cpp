/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alessandro Settimi, Marco Randazzo, and Daniele E. Domenichelli
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "yarp/dev/MultiCameraDriver.h"
#include "gazebo/MultiCameraLog.h"

#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>

#include <gazebo/sensors/MultiCameraSensor.hh>
#include <gazebo/rendering/Distortion.hh>

#include <yarp/os/Value.h>
#include <yarp/os/LogStream.h>

#include <algorithm>

const std::string YarpScopedName = "sensorScopedName";
double yarp::dev::GazeboYarpMultiCameraDriver::start_time = 0;

using GazeboYarpPlugins::GAZEBOMULTICAMERA;

static void print(unsigned char* pixbuf, int pixbuf_w, int pixbuf_h, int x, int y, char* s, int size)
{
    int pixelsize = 5;
    for (int i=0;i<size;i++) {
        const char* num_p=0;
        switch(s[i]) {
            case '0' : num_p = "**** ** ** ****"; break;
            case '1' : num_p = " *  *  *  *  * "; break;
            case '2' : num_p = "***  *****  ***"; break;
            case '3' : num_p = "***  ****  ****"; break;
            case '4' : num_p = "* ** ****  *  *"; break;
            case '5' : num_p = "****  ***  ****"; break;
            case '6' : num_p = "****  **** ****"; break;
            case '7' : num_p = "***  *  *  *  *"; break;
            case '8' : num_p = "**** ***** ****"; break;
            case '9' : num_p = "**** ****  ****"; break;
            case ' ' : num_p = "               "; break;
            case '.' : num_p = "          ** **"; break;
            default: break;
        }

        for (int yi = 0; yi < 5; yi++) {
            for (int xi = 0; xi < 3; xi++) {
                int ii = yi * 3 + xi;
                if (num_p[ii]=='*') {
                    for (int r = yi * pixelsize; r < yi * pixelsize + pixelsize; r++) {
                        int off = i * (pixelsize + 20);
                        for (int c = xi * pixelsize + off; c < xi * pixelsize + pixelsize + off; c++) {
                            unsigned char *pixel = pixbuf + c * 3 + r * (pixbuf_w * 3);
                            pixel[0] = 0;
                            pixel[1] = 0;
                            pixel[2] = 255;
                        }
                    }
                }
            }
        }
    }
}

yarp::dev::GazeboYarpMultiCameraDriver::GazeboYarpMultiCameraDriver() :
            m_max_height(0),
            m_max_width(0),
            m_vertical_flip(false),
            m_horizontal_flip(false),
            m_display_time_box(false),
            m_display_timestamp(false),
            m_counter(0)
{
    start_time = yarp::os::Time::now();
}


yarp::dev::GazeboYarpMultiCameraDriver::~GazeboYarpMultiCameraDriver()
{
    yTrace();
}

bool yarp::dev::GazeboYarpMultiCameraDriver::open(yarp::os::Searchable& config)
{
    //Get gazebo pointers
    std::string sensorScopedName(config.find(YarpScopedName.c_str()).asString().c_str());

    m_parentSensor = (gazebo::sensors::MultiCameraSensor*)GazeboYarpPlugins::Handler::getHandler()->getSensor(sensorScopedName);
    if (!m_parentSensor) {
        yCError(GAZEBOMULTICAMERA) << "GazeboYarpMultiCameraDriver Error: camera sensor was not found";
        return false;
    }

    m_vertical_flip     = config.check("vertical_flip");
    m_horizontal_flip   = config.check("horizontal_flip");
    m_display_timestamp = config.check("display_timestamp");
    m_display_time_box  = config.check("display_time_box");
    m_vertical          = config.check("vertical");

    m_camera_count = this->m_parentSensor->CameraCount();

    for (unsigned int i = 0; i < m_camera_count; ++i) {
        m_camera.push_back(m_parentSensor->Camera(i));

        if(m_camera[i] == NULL) {
            yCError(GAZEBOMULTICAMERA) << "GazeboYarpMultiCameraDriver: camera" << i <<  "pointer is not valid";
            return false;
       }
        m_width.push_back(m_camera[i]->ImageWidth());
        m_height.push_back(m_camera[i]->ImageHeight());

        m_max_width = std::max(m_max_width, m_width[i]);
        m_max_height = std::max(m_max_height, m_height[i]);

        m_bufferSize.push_back(3 * m_width[i] * m_height[i]);
        m_dataMutex.push_back(new std::mutex());
        {
            std::lock_guard<std::mutex> lock(*m_dataMutex[i]);
            m_imageBuffer.push_back(new unsigned char[m_bufferSize[i]]);
            memset(m_imageBuffer[i], 0x00, m_bufferSize[i]);
        }

        m_lastTimestamp.push_back(yarp::os::Stamp());
    }

    // Connect all the cameras only when everything is set up
    for (unsigned int i = 0; i < m_camera_count; ++i) {
        this->m_updateConnection.push_back(this->m_camera[i]->ConnectNewImageFrame(boost::bind(&yarp::dev::GazeboYarpMultiCameraDriver::captureImage, this, i, _1, _2, _3, _4, _5)));
    }

    return true;
}

bool yarp::dev::GazeboYarpMultiCameraDriver::close()
{
    for (unsigned int i = 0; i < m_camera_count; ++i) {
        this->m_updateConnection[i].reset();
        m_parentSensor = NULL;
        delete[] m_imageBuffer[i];
        m_imageBuffer[i] = 0;
        delete m_dataMutex[i];
    }

    return true;
}


bool yarp::dev::GazeboYarpMultiCameraDriver::captureImage(unsigned int _camera,
                                                          const unsigned char *_image,
                                                          unsigned int _width,
                                                          unsigned int _height,
                                                          unsigned int _depth,
                                                          const std::string &_format)
{
    std::lock_guard<std::mutex> lock(*m_dataMutex[_camera]);

    yAssert(_width  == m_width[_camera]);
    yAssert(_height == m_height[_camera]);
    // FIXME For now assume depth = 3 and format = "R8G8B8"
    yAssert(_depth  == 3);
    yAssert(_format == "R8G8B8");

    int rowsize = _depth * _width;

    if(m_parentSensor->IsActive()) {
        if (m_vertical_flip == false && m_horizontal_flip == false) {
            memcpy(m_imageBuffer[_camera], _image, m_bufferSize[_camera]);
        } else if (m_vertical_flip == true && m_horizontal_flip == false) {
           for (int r = 0; r < _height; r++) {
               memcpy(m_imageBuffer[_camera] + (rowsize*r), _image + rowsize * (_height-r), rowsize);
           }
        } else if (m_vertical_flip == false && m_horizontal_flip == true) {
            for (int r = 0; r < _height; r++) {
                for (int c = 0; c < _width; c++) {
                    memcpy(m_imageBuffer[_camera] + (rowsize*r) + (c*_depth), _image + rowsize * r + (_width-c-1) * _depth, _depth);
                }
            }
        } else { // (m_vertical_flip == true && m_horizontal_flip == true)
            for (int r = 0; r < _height; r++) {
                for (int c = 0; c < _width; c++) {
                    memcpy(m_imageBuffer[_camera] + (rowsize*r) + (c*_depth), _image + rowsize * (_height-r-1) + (_width-c-1) * _depth, _depth);
                }
            }
        }
    }

    m_lastTimestamp[_camera].update(this->m_parentSensor->LastUpdateTime().Double());

    if (m_display_timestamp) {
        char txtbuf[1000];
        double time=yarp::os::Time::now()-start_time;
        sprintf(txtbuf,"%.3f",time);
        int len = strlen(txtbuf);
        if (len < 20)
            print(m_imageBuffer[_camera], _width, _height, 0, 0, txtbuf, len);
    }

    return true;
}


bool yarp::dev::GazeboYarpMultiCameraDriver::getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& _image)
{
    for (unsigned int i = 0; i < m_camera_count; ++i) {
        m_dataMutex[i]->lock();
    }

    if (m_vertical) {
        _image.resize(m_max_width, m_max_height * m_camera_count);
    } else {
        _image.resize(m_max_width * m_camera_count, m_max_height);
    }

    unsigned char *pBuffer = _image.getRawImage();
    memset(pBuffer, 0, _image.getRawImageSize());

    for (unsigned int i = 0; i < m_camera_count; ++i) {

        int rowsize = 3 * m_width[i];
        int rowoffset = i * 3 * m_max_width;
        if (m_vertical) {
            if (m_max_width == m_width[i] && m_max_height == m_height[i]) {
                memcpy(pBuffer + (3 * m_max_width * m_max_height * i), m_imageBuffer[i], m_bufferSize[i]);
            } else {
                for (int r = 0; r < m_height[i]; r++) {
                    memcpy(pBuffer + (3 * m_max_width) * (m_max_height * i + r), m_imageBuffer[i] + rowsize * r, rowsize);
                }
            }
        } else {
            for (int r = 0; r < m_height[i]; r++) {
                memcpy(pBuffer + (3 * m_max_width * m_camera_count * r) + rowoffset, m_imageBuffer[i] + rowsize * r, rowsize);
            }
        }
    }

    if (m_display_time_box) {
        m_counter = (++m_counter % 10);
        for (int c = m_counter*30; c < 30 + m_counter*30; c++) {
            for (int r=0; r<30; r++) {
                for (unsigned int i = 0; i < m_camera_count; ++i) {
                    unsigned char *pixel;
                    if (m_vertical) {
                        pixel = _image.getPixelAddress(m_width[i]-c-1, m_max_height * i + m_height[i]-r-1);
                    } else {
                        pixel = _image.getPixelAddress(m_max_width * i + m_width[i]-c-1, m_height[i]-r-1);
                    }
                    pixel[0] = (m_counter % 2 == 0) ? 255 : 0;
                    pixel[1] = (m_counter % 2 == 0) ? 0 : 255;
                    pixel[2] = 0;
                }
            }
        }
    }

    for (unsigned int i = 0; i < m_camera_count; ++i) {
        m_dataMutex[i]->unlock();
    }

    return true;
}

int yarp::dev::GazeboYarpMultiCameraDriver::height() const
{
    return (m_vertical ? m_max_height * m_camera_count : m_max_height);
}

int yarp::dev::GazeboYarpMultiCameraDriver::width() const
{
    return (m_vertical ? m_max_width * m_camera_count : m_max_width * m_camera_count);
}

int yarp::dev::GazeboYarpMultiCameraDriver::getRgbHeight()
{
    return height();
}

int yarp::dev::GazeboYarpMultiCameraDriver::getRgbWidth()
{
    return width();
}

bool yarp::dev::GazeboYarpMultiCameraDriver::getRgbSupportedConfigurations(yarp::sig::VectorOf<yarp::dev::CameraConfig>& configurations)
{
    configurations.clear();
    yarp::dev::CameraConfig config {width(), height(), m_parentSensor->Camera(0)->RenderRate(), VOCAB_PIXEL_RGB};
    configurations.push_back(config);
    return true;
}

bool yarp::dev::GazeboYarpMultiCameraDriver::getRgbResolution(int& width, int& height)
{
    width = this->width();
    height = this->height();
    return true;
}

bool yarp::dev::GazeboYarpMultiCameraDriver::setRgbResolution(int width, int height)
{
    return false;
}

bool yarp::dev::GazeboYarpMultiCameraDriver::getRgbFOV(double& horizontalFov, double& verticalFov)
{
    using namespace gazebo::rendering;
    Camera* camPtr = m_parentSensor->Camera(0).get();
    horizontalFov = camPtr->HFOV().Degree();
    verticalFov = camPtr->VFOV().Degree();
    return true;
}

bool yarp::dev::GazeboYarpMultiCameraDriver::setRgbFOV(double horizontalFov, double verticalFov)
{
    return false;
}

bool yarp::dev::GazeboYarpMultiCameraDriver::getRgbIntrinsicParam(yarp::os::Property& intrinsic)
{
    using namespace gazebo::rendering;

    intrinsic.put("physFocalLength", 0.0);
    Camera* camPtr = m_parentSensor->Camera(0).get();
    if(camPtr)
    {
        intrinsic.put("focalLengthY",    1. / camPtr->OgreCamera()->getPixelDisplayRatio());
        intrinsic.put("focalLengthX",    1. / camPtr->OgreCamera()->getPixelDisplayRatio());
        Distortion* distModel = camPtr->LensDistortion().get();
        if(distModel)
        {
#if GAZEBO_MAJOR_VERSION >= 8
            intrinsic.put("k1",              distModel->K1());
            intrinsic.put("k2",              distModel->K2());
            intrinsic.put("k3",              distModel->K3());
            intrinsic.put("t1",              distModel->P1());
            intrinsic.put("t2",              distModel->P2());
            intrinsic.put("principalPointX", distModel->Center().X());
            intrinsic.put("principalPointY", distModel->Center().Y());
#else
            intrinsic.put("k1",              distModel->GetK1());
            intrinsic.put("k2",              distModel->GetK2());
            intrinsic.put("k3",              distModel->GetK3());
            intrinsic.put("t1",              distModel->GetP1());
            intrinsic.put("t2",              distModel->GetP2());
            intrinsic.put("principalPointX", distModel->GetCenter().x);
            intrinsic.put("principalPointY", distModel->GetCenter().y);
#endif
        }
        else
        {
            intrinsic.put("k1",              0.0);
            intrinsic.put("k2",              0.0);
            intrinsic.put("k3",              0.0);
            intrinsic.put("t1",              0.0);
            intrinsic.put("t2",              0.0);
            intrinsic.put("principalPointX", m_width[0]/2.0);
            intrinsic.put("principalPointY", m_height[0]/2.0);
        }
    }
    yarp::os::Value rectM;
    intrinsic.put("rectificationMatrix", rectM.makeList("1.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 1.0"));
    intrinsic.put("distortionModel", "plumb_bob");
    intrinsic.put("stamp", m_lastTimestamp[0].getTime());
    return true;
}

bool yarp::dev::GazeboYarpMultiCameraDriver::getRgbMirroring(bool& mirror)
{
    return false;
}

bool yarp::dev::GazeboYarpMultiCameraDriver::setRgbMirroring(bool mirror)
{
    return false;
}

yarp::os::Stamp yarp::dev::GazeboYarpMultiCameraDriver::getLastInputStamp()
{
    // FIXME Ensure that the images are syncronous
    for (unsigned int i = 1; i < m_camera_count; ++i) {
        if (m_lastTimestamp[i].getTime() != m_lastTimestamp[0].getTime()) {
            yCWarning(GAZEBOMULTICAMERA) << "Timestamp is different!" <<  m_lastTimestamp[0].getTime() << m_lastTimestamp[i].getTime();
        }
    }
    return m_lastTimestamp[0];
}
