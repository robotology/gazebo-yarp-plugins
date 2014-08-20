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

#include <boost/shared_ptr.hpp>

//Forward declarations
namespace yarp {
    namespace dev {
        class GazeboYarpIMUDriver;
    }
}

namespace gazebo {
    namespace common {
        class UpdateInfo;
    }
    namespace sensors {
        class ImuSensor;
    }
    namespace event {
        class Connection;
        typedef boost::shared_ptr<Connection> ConnectionPtr;
    }
}

extern const int YarpIMUChannelsNumber; //The IMU has 12 fixed channels
extern const std::string YarpIMUScopedName;

class yarp::dev::GazeboYarpIMUDriver:
    public yarp::dev::IGenericSensor,
    public yarp::dev::IPreciselyTimed,
    public yarp::dev::DeviceDriver
{
public:
    GazeboYarpIMUDriver();

    virtual ~GazeboYarpIMUDriver();
    
    void onUpdate(const gazebo::common::UpdateInfo&);

    /**
     * Yarp interfaces start here
     */

    //DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //GENERIC SENSOR
    virtual bool read(yarp::sig::Vector& outVector);
    virtual bool getChannels(int* numberOfChannels);
    virtual bool calibrate(int channelIndex, double v);
    
    //PRECISELY TIMED
    virtual yarp::os::Stamp getLastInputStamp();


private:
    yarp::sig::Vector m_imuData; //buffer for imu data
    yarp::os::Stamp m_lastTimestamp; //buffer for last timestamp data
    yarp::os::Semaphore m_dataMutex; //mutex for accessing the data
    
    gazebo::sensors::ImuSensor* m_parentSensor;
    gazebo::event::ConnectionPtr m_updateConnection;

};

#endif // GAZEBOYARP_IMUDRIVER_H
