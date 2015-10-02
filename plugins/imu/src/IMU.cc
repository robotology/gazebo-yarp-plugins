/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "IMU.hh"
#include "IMUDriver.h"
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>


#include <gazebo/sensors/ImuSensor.hh>

#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>

GZ_REGISTER_SENSOR_PLUGIN(gazebo::GazeboYarpIMU)

namespace gazebo {

GazeboYarpIMU::GazeboYarpIMU() : SensorPlugin()
{
}

GazeboYarpIMU::~GazeboYarpIMU()
{
    std::cout << "*** GazeboYarpIMU closing ***" << std::endl;
    m_imuDriver.close();
    GazeboYarpPlugins::Handler::getHandler()->removeSensor(m_sensorName);
    yarp::os::Network::fini();
}

void GazeboYarpIMU::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    yarp::os::Network::init();
    if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
        std::cerr << "GazeboYarpIMU::Load error: yarp network does not seem to be available, is the yarpserver running?"<<std::endl;
        return;
    }

    if (!_sensor) {
        gzerr << "GazeboYarpIMU plugin requires a IMUSensor." << std::endl;
        return;
    }

    _sensor->SetActive(true);

    // Add my gazebo device driver to the factory.
    ::yarp::dev::Drivers::factory().add(new ::yarp::dev::DriverCreatorOf< ::yarp::dev::GazeboYarpIMUDriver>
                                        ("gazebo_imu", "inertial", "GazeboYarpIMUDriver"));

    bool configuration_loaded = GazeboYarpPlugins::loadConfigSensorPlugin(_sensor,_sdf,m_parameters);

    if (!configuration_loaded) {
        return;
    }

    m_sensorName = _sensor->GetScopedName();
    //Insert the pointer in the singleton handler for retriving it in the yarp driver
    GazeboYarpPlugins::Handler::getHandler()->setSensor(_sensor.get());

    m_parameters.put(YarpIMUScopedName.c_str(), m_sensorName.c_str());

    //Open the driver
    if (m_imuDriver.open(m_parameters)) {
    } else {
        std::cout << "GazeboYarpIMU Plugin Load failed: error in opening yarp driver" << std::endl;
    }
}

}
