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
#include <yarp/dev/Lidar2DDeviceBase.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <boost/shared_ptr.hpp>

#include <gazebo/common/Plugin.hh>

#include <mutex>

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

extern const std::string YarpLaserSensorScopedName;

class yarp::dev::GazeboYarpLaserSensorDriver: 
    public yarp::dev::Lidar2DDeviceBase,
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
    virtual bool open(yarp::os::Searchable& config) override;
    virtual bool close() override;

    //IRangefinder2D
    virtual bool setDistanceRange (double min, double max) override;
    virtual bool setScanLimits (double min, double max) override;
    virtual bool setHorizontalResolution (double step) override;
    virtual bool setScanRate (double rate) override;

    //PRECISELY TIMED
    // TODO(traversaro): Remove once we require YARP 3.6
    // See https://github.com/robotology/gazebo-yarp-plugins/issues/598
    virtual yarp::os::Stamp getLastInputStamp();

public:
    //Lidar2DDeviceBase
    bool acquireDataFromHW() override final;
    
private:
    double m_gazebo_max_angle;
    double m_gazebo_min_angle;
    double m_gazebo_max_range;
    double m_gazebo_min_range;
    double m_gazebo_resolution;
    size_t m_gazebo_samples;
    double m_gazebo_scan_rate;
    bool   m_first_run;
    
    gazebo::sensors::RaySensor* m_parentSensor;
    gazebo::event::ConnectionPtr m_updateConnection;

};

#endif // GAZEBOYARP_LASERSENSORDRIVER_H
