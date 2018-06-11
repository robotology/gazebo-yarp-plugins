/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_LASERSENSORDRIVER_H
#define GAZEBOYARP_LASERSENSORDRIVER_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IRangefinder2D.h>
#include <yarp/dev/LaserMeasurementData.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/Semaphore.h>
#include <boost/shared_ptr.hpp>

#include <gazebo/common/Plugin.hh>

namespace yarp {
    namespace dev {
        class GazeboYarpLaserSensorDriver;
    }
}

namespace gazebo {
    namespace common {
        class UpdateInfo;
    }
    namespace sensors {
        class RaySensor;
    }
    namespace event {
        class Connection;
        typedef boost::shared_ptr<Connection> ConnectionPtr;
    }
}

struct Range_t
{
    double min;
    double max;
};

extern const std::string YarpLaserSensorScopedName;

class yarp::dev::GazeboYarpLaserSensorDriver: 
    public yarp::dev::IRangefinder2D,
    public yarp::dev::IPreciselyTimed,
    public yarp::dev::DeviceDriver
{
public:
    GazeboYarpLaserSensorDriver();
    virtual ~GazeboYarpLaserSensorDriver();
    
    void onUpdate(const gazebo::common::UpdateInfo& /*_info*/);

    /**
     * Yarp interfaces start here
     */

    //DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //ANALOG SENSOR
    virtual bool getLaserMeasurement (std::vector<yarp::dev::LaserMeasurementData> &data);
    virtual bool getRawData (yarp::sig::Vector &data);
    virtual bool getDeviceStatus (Device_status &status);
    virtual bool getDistanceRange (double &min, double &max);
    virtual bool setDistanceRange (double min, double max);
    virtual bool getScanLimits (double &min, double &max);
    virtual bool setScanLimits (double min, double max);
    virtual bool getHorizontalResolution (double &step);
    virtual bool setHorizontalResolution (double step);
    virtual bool getScanRate (double &rate);
    virtual bool setScanRate (double rate);
    virtual bool getDeviceInfo (std::string &device_info);

    //PRECISELY TIMED
    virtual yarp::os::Stamp getLastInputStamp();


private:
    double m_max_angle; 
    double m_min_angle; 
    double m_max_clip_range; 
    double m_min_clip_range; 
    double m_max_discard_range; 
    double m_min_discard_range;
    double m_max_gazebo_range; 
    double m_min_gazebo_range;
    double m_samples;  
    double m_resolution;
    double m_rate; 
    bool   m_enable_clip_range;
    bool   m_enable_discard_range;
    bool   m_first_run;
    
    Device_status m_device_status;
    std::vector <Range_t> range_skip_vector;
    
    std::vector<double> m_sensorData; //buffer for laser data
    yarp::os::Stamp m_lastTimestamp; //buffer for last timestamp data
    yarp::os::Mutex m_mutex; //mutex for accessing the data
    gazebo::sensors::RaySensor* m_parentSensor;
    gazebo::event::ConnectionPtr m_updateConnection;

};

#endif // GAZEBOYARP_LASERSENSORDRIVER_H
