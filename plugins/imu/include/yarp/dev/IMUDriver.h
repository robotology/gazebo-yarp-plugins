/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_IMUDRIVER_H
#define GAZEBOYARP_IMUDRIVER_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/GenericSensorInterfaces.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/IPreciselyTimed.h>

#include <boost/shared_ptr.hpp>

#include <mutex>

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

extern const unsigned YarpIMUChannelsNumber; //The IMU has 12 fixed channels
extern const std::string YarpIMUScopedName;

class yarp::dev::GazeboYarpIMUDriver:
    public yarp::dev::IGenericSensor,
    public yarp::dev::IPreciselyTimed,
    public yarp::dev::DeviceDriver,
    public yarp::dev::IThreeAxisGyroscopes,
    public yarp::dev::IThreeAxisLinearAccelerometers,
    public yarp::dev::IThreeAxisMagnetometers,
    public yarp::dev::IOrientationSensors
{
public:
    GazeboYarpIMUDriver();

    virtual ~GazeboYarpIMUDriver();

    void onUpdate(const gazebo::common::UpdateInfo&);

    /**
     * Yarp interfaces start here
     */

    //DEVICE DRIVER
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    //GENERIC SENSOR
    bool read(yarp::sig::Vector& outVector) override;
    bool getChannels(int* numberOfChannels) override;
    bool calibrate(int channelIndex, double v) override;

    //PRECISELY TIMED
    yarp::os::Stamp getLastInputStamp() override;

    /* IThreeAxisGyroscopes methods */

    size_t getNrOfThreeAxisGyroscopes() const override;

    yarp::dev::MAS_status getThreeAxisGyroscopeStatus(size_t sens_index) const override;

    bool getThreeAxisGyroscopeName(size_t sens_index, std::string &name) const override;

    bool getThreeAxisGyroscopeFrameName(size_t sens_index, std::string &frameName) const override;

    bool getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IThreeAxisLinearAccelerometers methods */

    size_t getNrOfThreeAxisLinearAccelerometers() const override;

    yarp::dev::MAS_status getThreeAxisLinearAccelerometerStatus(size_t sens_index) const override;

    bool getThreeAxisLinearAccelerometerName(size_t sens_index, std::string &name) const override;

    bool getThreeAxisLinearAccelerometerFrameName(size_t sens_index, std::string &frameName) const override;

    bool getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IThreeAxisMagnetometers methods */

    size_t getNrOfThreeAxisMagnetometers() const override;

    yarp::dev::MAS_status getThreeAxisMagnetometerStatus(size_t sens_index) const override;

    bool getThreeAxisMagnetometerName(size_t sens_index, std::string &name) const override;

    bool getThreeAxisMagnetometerFrameName(size_t sens_index, std::string &frameName) const override;

    bool getThreeAxisMagnetometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IOrientationSensors methods */

    size_t getNrOfOrientationSensors() const override;

    yarp::dev::MAS_status getOrientationSensorStatus(size_t sens_index) const override;

    bool getOrientationSensorName(size_t sens_index, std::string &name) const override;

    bool getOrientationSensorFrameName(size_t sens_index, std::string &frameName) const override;

    bool getOrientationSensorMeasureAsRollPitchYaw(size_t sens_index, yarp::sig::Vector& rpy, double& timestamp) const override;


private:

    yarp::dev::MAS_status genericGetStatus(size_t sens_index) const;
    bool genericGetSensorName(size_t sens_index, std::string &name) const;
    bool genericGetFrameName(size_t sens_index, std::string &frameName) const;
    bool genericGetMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp, size_t startIdx) const;

    yarp::sig::Vector m_imuData; //buffer for imu data
    yarp::os::Stamp m_lastTimestamp; //buffer for last timestamp data
    mutable std::mutex m_dataMutex; //mutex for accessing the data
    std::string m_sensorName{"sensor_imu_gazebo"};
    std::string m_frameName{"sensor_imu_gazebo"};

    gazebo::sensors::ImuSensor* m_parentSensor{};
    gazebo::event::ConnectionPtr m_updateConnection;

};

#endif // GAZEBOYARP_IMUDRIVER_H
