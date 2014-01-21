/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef __GAZEBO_YARP_FORCETORQUE_DRIVER_H__
#define __GAZEBO_YARP_FORCETORQUE_DRIVER_H__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/Semaphore.h>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/ForceTorqueSensor.hh>


namespace yarp {
    namespace dev {
        class GazeboYarpForceTorqueDriver;
    }
}

const int yarp_forcetorque_nr_of_channels = 6; //The IMU has 6 fixed channels

const std::string yarp_scopedname_parameter = "sensorScopedName";

class yarp::dev::GazeboYarpForceTorqueDriver: 
    public yarp::dev::IAnalogSensor,
    public yarp::dev::IPreciselyTimed,
    public yarp::dev::DeviceDriver
{
public:
    GazeboYarpForceTorqueDriver();

    ~GazeboYarpForceTorqueDriver();
    
    void onUpdate(const gazebo::common::UpdateInfo & /*_info*/);

    /**
     * Yarp interfaces start here
     */
    
    //DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config);    
    virtual bool close();
    
    //ANALOG SENSOR
    virtual int read(yarp::sig::Vector &out);
    virtual int getState(int ch);
    virtual int getChannels();
    virtual int calibrateChannel(int ch, double v);
    virtual int calibrateSensor();
    virtual int calibrateSensor(const yarp::sig::Vector& value);
    virtual int calibrateChannel(int ch);
    
    //PRECISELY TIMED
    virtual yarp::os::Stamp getLastInputStamp();


private:
    yarp::sig::Vector forcetorque_data; //buffer for forcetorque sensor data
    
    yarp::os::Stamp last_timestamp; //buffer for last timestamp data
    
    yarp::os::Semaphore data_mutex; //mutex for accessing the data
    
    gazebo::sensors::ForceTorqueSensor* parentSensor;
    
    gazebo::event::ConnectionPtr updateConnection;


};

#endif // __GAZEBO_YARP_FORCETORQUE_DRIVER_H__
