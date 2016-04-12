/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alessandro Settimi, Marco Randazzo, and Daniele E. Domenichelli
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_MULTICAMERADRIVER_H
#define GAZEBOYARP_MULTICAMERADRIVER_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>

#include <boost/shared_ptr.hpp>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/sensors/MultiCameraSensor.hh>


//Forward declarations
namespace yarp {
    namespace dev {
        class GazeboYarpMultiCameraDriver;
    }
}

namespace gazebo {
    namespace common {
        class UpdateInfo;
    }
    namespace sensors {
        class MultiCameraSensor;
    }
    namespace event {
        class Connection;
        typedef boost::shared_ptr<Connection> ConnectionPtr;
    }
}

extern const std::string YarpScopedName;

class yarp::dev::GazeboYarpMultiCameraDriver:
    virtual public yarp::dev::DeviceDriver,
    virtual public yarp::dev::IFrameGrabberImage,
    virtual public yarp::dev::IPreciselyTimed
{
public:
    GazeboYarpMultiCameraDriver();

    virtual ~GazeboYarpMultiCameraDriver();

    /**
     * Yarp interfaces start here
     */

    // yarp::dev::DeviceDriver
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    // yarp::dev::IPreciselyTimed
    virtual yarp::os::Stamp getLastInputStamp();

    // yarp::dev::IFrameGrabberImage
    virtual bool getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image);
    virtual int height() const;
    virtual int width() const;


    /*
     * Get the image from the simulator and store it internally
     */
    virtual bool captureImage(unsigned int _camera,
                              const unsigned char *_image,
                              unsigned int _width,
                              unsigned int _height,
                              unsigned int _depth,
                              const std::string &_format);

private:
    static double start_time;

    unsigned int m_camera_count;

    std::vector<int> m_width;
    std::vector<int> m_height;
    std::vector<int> m_bufferSize;

    int m_max_width;
    int m_max_height;

    bool m_vertical_flip;
    bool m_horizontal_flip;
    bool m_display_time_box;
    bool m_display_timestamp;
    bool m_vertical;

    std::vector<yarp::os::Stamp> m_lastTimestamp; // buffer for last timestamp data
    std::vector<yarp::os::Semaphore*> m_dataMutex; // mutex for accessing the data

    std::vector<unsigned char*> m_imageBuffer;
    int m_counter;

    gazebo::sensors::MultiCameraSensor* m_parentSensor;
    std::vector<gazebo::rendering::CameraPtr> m_camera;
    std::vector<gazebo::event::ConnectionPtr> m_updateConnection;
};

#endif // GAZEBOYARP_MULTICAMERADRIVER_H
