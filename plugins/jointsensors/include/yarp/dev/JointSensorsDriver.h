/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_JOINTSENSORSDRIVER_HH
#define GAZEBOYARP_JOINTSENSORSDRIVER_HH

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/Semaphore.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>

namespace yarp {
    namespace dev {
        class GazeboYarpJointSensorsDriver;
    }
}


class yarp::dev::GazeboYarpJointSensorsDriver:
    public yarp::dev::IAnalogSensor,
    public yarp::dev::IPreciselyTimed,
    public yarp::dev::DeviceDriver
{
public:

    GazeboYarpJointSensorsDriver();

    virtual ~GazeboYarpJointSensorsDriver();

    /**
     * Gazebo stuff
     */
    bool gazebo_init();
    void onUpdate(const gazebo::common::UpdateInfo & /*_info*/);

    /**
     * Yarp interfaces implementation
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
    enum {
        Position,
        Speed,
        Torque
    } jointsensors_type;

    gazebo::physics::Model* _robot;

    std::vector<gazebo::physics::Joint *> joint_ptrs;

    yarp::sig::Vector jointsensors_data; //buffer for joint sensors data

    int jointsensors_nr_of_channels;

    yarp::os::Stamp last_timestamp; //buffer for last timestamp data

    yarp::os::Semaphore data_mutex; //mutex for accessing the data

    gazebo::event::ConnectionPtr updateConnection;

    bool setJointPointers(yarp::os::Property & plugin_parameters);
    bool setJointSensorsType(yarp::os::Property & plugin_parameters);

};

#endif //GAZEBOYARP_JOINTSENSORSDRIVER_HH

