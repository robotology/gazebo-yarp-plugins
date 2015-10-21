/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alessandro Settimi and Marco Randazzo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_CAMERADRIVER_H
#define GAZEBOYARP_CAMERADRIVER_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>

#include <boost/shared_ptr.hpp>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/sensors/CameraSensor.hh>


//Forward declarations
namespace yarp {
    namespace dev {
        class GazeboYarpCameraDriver;
    }
}

namespace gazebo {
    namespace common {
        class UpdateInfo;
    }
    namespace sensors {
        class CameraSensor;
    }
    namespace event {
        class Connection;
        typedef boost::shared_ptr<Connection> ConnectionPtr;
    }
}

extern const std::string YarpScopedName;

class yarp::dev::GazeboYarpCameraDriver:
    virtual public yarp::dev::DeviceDriver,
    virtual public yarp::dev::IFrameGrabberImage,
    virtual public yarp::dev::IPreciselyTimed
{
public:
    GazeboYarpCameraDriver();

    virtual ~GazeboYarpCameraDriver();

    /**
     * Yarp interfaces start here
     */

    //DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //PRECISELY TIMED
    virtual yarp::os::Stamp getLastInputStamp();


    // IFRAMEGRABBER IMAGE
    /**
     * Get an rgb image from the frame grabber, if required
     * demosaicking/color reconstruction is applied
     *
     * @param image the image to be filled
     * @return true/false upon success/failure
     */
    virtual bool getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& image);

    /**
     * Return the height of each frame.
     * @return image height
     */
    virtual int height() const;

    /**
     * Return the width of each frame.
     * @return image width
     */
    virtual int width() const;


    /*
     * Get the image from the simulator and store it internally
     */
    virtual bool captureImage(const unsigned char *_image,
                              unsigned int _width, unsigned int _height,
                              unsigned int _depth, const std::string &_format);

    virtual int getRawBufferSize();
    void print (unsigned char* pixbuf, int pixbuf_w, int pixbuf_h, int x, int y, char* s, int size);

private:
    // camera data here
    int m_width;
    int m_height;
    int m_bufferSize;
    bool m_vertical_flip;
    bool m_horizontal_flip;
    static double start_time;
    bool m_display_time_box;
    bool m_display_timestamp;

    yarp::os::Stamp m_lastTimestamp; //buffer for last timestamp data
    yarp::os::Semaphore m_dataMutex; //mutex for accessing the data

    unsigned char *m_imageBuffer;
    int counter;

    gazebo::sensors::CameraSensor* m_parentSensor;
    gazebo::rendering::CameraPtr m_camera;
    gazebo::event::ConnectionPtr m_updateConnection;
    
    struct txt_type
    {
       char data[16];
    };
    txt_type num[12];

};

#endif // GAZEBOYARP_CAMERADRIVER_H
