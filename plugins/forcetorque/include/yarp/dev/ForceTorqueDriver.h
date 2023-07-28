/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_FORCETORQUEDRIVER_H
#define GAZEBOYARP_FORCETORQUEDRIVER_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <boost/shared_ptr.hpp>

#include <gazebo/common/Plugin.hh>

#include <mutex>

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
    public yarp::dev::DeviceDriver,
    public yarp::dev::ISixAxisForceTorqueSensors,
    public yarp::dev::ITemperatureSensors
{
public:
    GazeboYarpForceTorqueDriver();
    virtual ~GazeboYarpForceTorqueDriver();
    
    void onUpdate(const gazebo::common::UpdateInfo& /*_info*/);

    /**
     * Yarp interfaces start here
     */

    //DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config) override;
    virtual bool close() override;

    //ANALOG SENSOR
    virtual int read(yarp::sig::Vector& out) override;
    virtual int getState(int channel) override;
    virtual int getChannels() override;
    virtual int calibrateChannel(int channel, double v) override;
    virtual int calibrateSensor() override;
    virtual int calibrateSensor(const yarp::sig::Vector& value) override;
    virtual int calibrateChannel(int channel) override;

    // SIX AXIS FORCE TORQUE SENSORS
    virtual size_t getNrOfSixAxisForceTorqueSensors() const override;
    virtual yarp::dev::MAS_status getSixAxisForceTorqueSensorStatus(size_t sens_index) const override;
    virtual bool getSixAxisForceTorqueSensorName(size_t sens_index, std::string &name) const override;
    virtual bool getSixAxisForceTorqueSensorFrameName(size_t sens_index, std::string &frameName) const override;
    virtual bool getSixAxisForceTorqueSensorMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    // TEMPERATURE SENSORS
    virtual size_t getNrOfTemperatureSensors() const override;
    virtual yarp::dev::MAS_status getTemperatureSensorStatus(size_t sens_index) const override;
    virtual bool getTemperatureSensorName(size_t sens_index, std::string &name) const override;
    virtual bool getTemperatureSensorFrameName(size_t sens_index, std::string &frameName) const override;
    virtual bool getTemperatureSensorMeasure(size_t sens_index, double& out, double& timestamp) const override;
    virtual bool getTemperatureSensorMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    //PRECISELY TIMED
    virtual yarp::os::Stamp getLastInputStamp() override;


private:
    yarp::sig::Vector m_forceTorqueData; //buffer for forcetorque sensor data
    yarp::os::Stamp m_lastTimestamp; //buffer for last timestamp data
    mutable std::mutex m_dataMutex; //mutex for accessing the data
    gazebo::sensors::ForceTorqueSensor* m_parentSensor;
    gazebo::event::ConnectionPtr m_updateConnection;
    std::string m_sensorName;
    std::string m_frameName;

};

#endif // GAZEBO_YARP_FORCETORQUE_DRIVER_H
