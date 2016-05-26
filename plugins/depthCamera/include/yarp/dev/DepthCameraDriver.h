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
public:
    GazeboYarpDepthCameraDriver();

    virtual ~GazeboYarpDepthCameraDriver();

    /**
     * Yarp interfaces start here
     */

    //DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    // IFRAMEGRABBER IMAGE
    /**
     * Get an rgb image from the frame grabber, if required
     * demosaicking/color reconstruction is applied
     *
     * @param image the image to be filled
     * @return true/false upon success/failure
     */
    virtual bool getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb> &image);
    virtual bool getImage(yarp::sig::ImageOf<yarp::sig::PixelMono> &image);
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


    // IRGBDSensor interface
    virtual bool getDeviceInfo(yarp::os::Searchable *device_info);

   /**
    * Get the distance measurements as an image
    * @param ranges the vector containing the distance measurement
    * @return true if able to get measurement data.
    */
    virtual bool getMeasurementData(yarp::sig::FlexImage &image, yarp::os::Stamp *stamp=NULL);

   /**
    * get the device status
    * @param status the device status
    * @return true/false.
    */
    virtual bool getDeviceStatus(DepthSensor_status *status);

   /**
    * get the device detection range
    * @param min the minimum detection distance from the sensor [meter]
    * @param max the maximum detection distance from the sensor [meter]
    * @return true if able to get required info.
    */
    virtual bool getDistanceRange(double *min, double *max);

   /**
    * set the device detection range. Invalid setting will be discarded.
    * @param min the minimum detection distance from the sensor [meter]
    * @param max the maximum detection distance from the sensor [meter]
    * @return true if message was correctly delivered to the HW device.
    */
    virtual bool setDistanceRange(double min, double max);

   /**
    * get the horizontal scan limits / field of view with respect to the
    * front line of sight of the sensor. Angles are measured around the
    * positive Z axis (counterclockwise, if Z is up) with zero angle being
    * forward along the x axis
    * @param min start angle of the scan  [degrees]
    * @param max end angle of the scan    [degrees]
    * @return true if able to get required info.
    */
    virtual bool getHorizontalScanLimits(double *min, double *max);

   /**
    * set the horizontal scan limits / field of view with respect to the
    * front line of sight of the sensor. Angles are measured around the
    * positive Z axis (counterclockwise, if Z is up) with zero angle being
    * forward along the x axis
    * @param min start angle of the scan  [degrees]
    * @param max end angle of the scan    [degrees]
    * @return true if message was correctly delivered to the HW device.
    */
    virtual bool setHorizontalScanLimits(double min, double max);

   /**
    * get the vertical scan limits / field of view with respect to the
    * front line of sight of the sensor   [degrees]
    * @param min start angle of the scan  [degrees]
    * @param max end angle of the scan    [degrees]
    * @return true if able to get required info.
    */
    virtual bool getVerticalScanLimits(double *min, double *max);

   /**
    * set the vertical scan limits / field of view with respect to the
    * front line of sight of the sensor   [degrees]
    * @param min start angle of the scan  [degrees]
    * @param max end angle of the scan    [degrees]
    * @return true if message was correctly delivered to the HW device.
    */
    virtual bool setVerticalScanLimits(double min, double max);

   /**
    * get the size of measured data from the device.
    * It can be WxH for camera-like devices, or the number of points for other devices.
    * @param horizontal width of image,  number of points in the horizontal scan [num]
    * @param vertical   height of image, number of points in the vertical scan [num]
    * @return true if able to get required info.
    */
    virtual bool getDataSize(double *horizontal, double *vertical);

   /**
    * set the size of measured data from the device.
    * It can be WxH for camera-like devices, or the number of points for other devices.
    * @param horizontal width of image,  number of points in the horizontal scan [num]
    * @param vertical   height of image, number of points in the vertical scan [num]
    * @return true if message was correctly delivered to the HW device.
    */
    virtual bool setDataSize(double horizontal, double vertical);

   /**
    * get the device resolution, using the current settings of scan limits
    * and data size. Will return the resolution of device at 1 meter distance.
    * @param hRes horizontal resolution [meter]
    * @param vRes vertical resolution [meter]
    * @return true if able to get required info.
    */
    virtual bool getResolution(double *hRes, double *vRes);

   /**
    * set the device resolution.
    * This call can change the current settings of scan limits, data size or scan rate
    * to match the requested resolution.
    * Verify those settings is suggested after this call.
    * Will set the resolution of device at 1meter distance, if possible.
    * @param hRes horizontal resolution [meter]
    * @param vRes vertical resolution [meter]
    * @return true if message was correctly delivered to the HW device.
    */
    virtual bool setResolution(double hRes, double vRes);

   /**
    * get the scan rate (scans per seconds)
    * @param rate the scan rate
    * @return true if able to get required info.
    */
    virtual bool getScanRate(double *rate);

   /**
    * set the scan rate (scans per seconds)
    * @param rate the scan rate
    * @return true if message was correctly delivered to the HW device.
    */
    virtual bool setScanRate(double rate);

    virtual bool getRGBDSensor_Status(RGBDSensor_status *status);

    /**
    * Get the both the color and depth frame in a single call. Implementation should assure the best possible synchronization
    * is achieved accordingly to synch policy set by the user.
    * TimeStamps are referred to acquisition time of the corresponding piece of information.
    * If the device is not providing TimeStamps, then 'timeStamp' field should be set to '-1'.
    * @param colorFrame pointer to FlexImage data to hold the color frame from the sensor
    * @param depthFrame pointer to FlexImage data to hold the depth frame from the sensor
    * @param colorStamp pointer to memory to hold the Stamp of the color frame
    * @param depthStamp pointer to memory to hold the Stamp of the depth frame
    * @return true if able to get both data.
    */
    virtual bool getRGBD_Frames(yarp::sig::FlexImage &colorFrame, yarp::sig::FlexImage &depthFrame, yarp::os::Stamp *colorStamp=NULL, yarp::os::Stamp *depthStamp=NULL);


    /*
     * INTERFACE TOWARD GAZEBO SIMULATOR CORE
     * Get the image from the simulator and store it internally
     */
    bool OnNewImageFrame(const   unsigned char *_image,
                                 unsigned int _width, unsigned int _height,
                                 unsigned int _depth, const std::string &_format);

    void OnNewRGBPointCloud(const   float * /*_pcd*/,
                                    unsigned int /*_width*/, unsigned int /*_height*/,
                                    unsigned int /*_depth*/, const std::string &/*_format*/);

    void OnNewDepthFrame(const  float * /*_image*/,
                                unsigned int /*_width*/,
                                unsigned int /*_height*/,
                                unsigned int /*_depth*/,
                                const std::string &/*_format*/);

    virtual int getRawBufferSize();

private:
    // camera data here
    int m_width;
    int m_height;

    int m_imageFrame_BufferSize;
    int m_depthFrame_BufferSize;
    int m_RGBPointCloud_BufferSize;

    bool m_vertical_flip;
    bool m_horizontal_flip;
    bool m_display_time_box;
    bool m_display_timestamp;

    yarp::os::Stamp m_colorTimestamp; // last timestamp data
    yarp::os::Stamp m_depthTimestamp; // last timestamp data
    yarp::os::Semaphore m_colorFrameMutex;  //mutex for accessing the data
    yarp::os::Semaphore m_depthFrameMutex;  //mutex for accessing the data

    unsigned char  *m_imageFrame_Buffer;
    float          *m_depthFrame_Buffer;
    unsigned char  *m_RGBPointCloud_Buffer;
    int counter;

#if GAZEBO_MAJOR_VERSION < 7
    gazebo::sensors::CameraSensor*       m_imageCameraSensorPtr;
    gazebo::rendering::CameraPtr         m_imageCameraPtr;
#endif
    gazebo::sensors::DepthCameraSensor*  m_depthCameraSensorPtr;
    gazebo::rendering::DepthCameraPtr    m_depthCameraPtr;

    gazebo::event::ConnectionPtr m_updateDepthFrame_Connection;
    gazebo::event::ConnectionPtr m_updateRGBPointCloud_Connection;
    gazebo::event::ConnectionPtr m_updateImageFrame_Connection;
};

#endif // GAZEBOYARP_DEPTHCAMERADRIVER_H
