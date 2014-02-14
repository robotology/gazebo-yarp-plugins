/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef GAZEBOYARP_IMUDRIVER_H
#define GAZEBOYARP_IMUDRIVER_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/GenericSensorInterfaces.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/Semaphore.h>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/ImuSensor.hh>


namespace yarp {
    namespace dev {
        class GazeboYarpIMUDriver;
    }
}

const int yarp_imu_nr_of_channels = 12; //The IMU has 12 fixed channels

const std::string yarp_scopedname_parameter = "sensorScopedName";

class yarp::dev::GazeboYarpIMUDriver: 
    public yarp::dev::IGenericSensor,
    public yarp::dev::IPreciselyTimed,
    public yarp::dev::DeviceDriver
{
public:
    GazeboYarpIMUDriver();

    virtual ~GazeboYarpIMUDriver();
    
    void onUpdate(const gazebo::common::UpdateInfo & /*_info*/);

    /**
     * Yarp interfaces start here
     */
    
    //DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config);    
    virtual bool close();
    
    //GENERIC SENSOR
    virtual bool read(yarp::sig::Vector &out);
    virtual bool getChannels(int *nc);
    virtual bool calibrate(int ch, double v);
    
    //PRECISELY TIMED
    virtual yarp::os::Stamp getLastInputStamp();


private:
    yarp::sig::Vector imu_data; //buffer for imu data
    
    yarp::os::Stamp last_timestamp; //buffer for last timestamp data
    
    yarp::os::Semaphore data_mutex; //mutex for accessing the data
    
    gazebo::sensors::ImuSensor* parentSensor;
    
    gazebo::event::ConnectionPtr updateConnection;


};

#endif // GAZEBOYARP_IMUDRIVER_H
