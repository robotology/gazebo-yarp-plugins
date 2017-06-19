/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_FORCETORQUEDRIVER_H
#define GAZEBOYARP_FORCETORQUEDRIVER_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/Semaphore.h>
#include <boost/shared_ptr.hpp>

#include <gazebo/common/Plugin.hh>

namespace yarp {
    namespace dev {
        class GazeboYarpForceTorqueDriver;
    }
}

namespace gazebo {
    namespace common {
        class UpdateInfo;
    }
    namespace sensors {
        class ForceTorqueSensor;
    }
    namespace event {
        class Connection;
        typedef boost::shared_ptr<Connection> ConnectionPtr;
    }
}


extern const unsigned YarpForceTorqueChannelsNumber;
extern const std::string YarpForceTorqueScopedName;

/**
 * YARP Force Torque Driver for Gazebo
 *
 * This driver does not accept any parameter, see
 * gazebo::GazeboYarpForceTorque for details on how to use it.
 */
class yarp::dev::GazeboYarpForceTorqueDriver: 
    public yarp::dev::IAnalogSensor,
    public yarp::dev::IPreciselyTimed,
    public yarp::dev::DeviceDriver
{
public:
    GazeboYarpForceTorqueDriver();
    virtual ~GazeboYarpForceTorqueDriver();
    
    void onUpdate(const gazebo::common::UpdateInfo& /*_info*/);

    /**
     * Yarp interfaces start here
     */

    //DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //ANALOG SENSOR
    virtual int read(yarp::sig::Vector& out);
    virtual int getState(int channel);
    virtual int getChannels();
    virtual int calibrateChannel(int channel, double v);
    virtual int calibrateSensor();
    virtual int calibrateSensor(const yarp::sig::Vector& value);
    virtual int calibrateChannel(int channel);

    //PRECISELY TIMED
    virtual yarp::os::Stamp getLastInputStamp();


private:
    yarp::sig::Vector m_forceTorqueData; //buffer for forcetorque sensor data
    yarp::os::Stamp m_lastTimestamp; //buffer for last timestamp data
    yarp::os::Semaphore m_dataMutex; //mutex for accessing the data
    gazebo::sensors::ForceTorqueSensor* m_parentSensor;
    gazebo::event::ConnectionPtr m_updateConnection;

};

#endif // GAZEBO_YARP_FORCETORQUE_DRIVER_H
