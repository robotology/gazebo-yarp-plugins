/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include "IMU.hh"
#include "IMUDriver.h"
#include "Handler.hh"
#include "common.h"

#include <gazebo/sensors/ImuSensor.hh>

#include <yarp/dev/ServerInertial.h>
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
    
    std::cout << "*** GazeboYarpIMU plugin started ***" << std::endl;
    
    if (!_sensor) {
        gzerr << "GazeboYarpIMU plugin requires a IMUSensor." << std::endl;
        return;
    }
    
    _sensor->SetActive(true);
    
    // Add my gazebo device driver to the factory.
    ::yarp::dev::Drivers::factory().add(new ::yarp::dev::DriverCreatorOf< ::yarp::dev::GazeboYarpIMUDriver>
                                        ("gazebo_imu", "inertial", "GazeboYarpIMUDriver"));
    
    
    //Getting .ini configuration file from sdf
    bool configuration_loaded = false;
    
    if (_sdf->HasElement("yarpConfigurationFile")) {
        std::string ini_file_name = _sdf->Get<std::string>("yarpConfigurationFile");
        std::string ini_file_path = gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);
        
        if (ini_file_path != "" && m_parameters.fromConfigFile(ini_file_path.c_str())) {
            std::cout << "Found yarpConfigurationFile: loading from " << ini_file_path << std::endl;
            configuration_loaded = true;
        }
    }
    
    if (!configuration_loaded) {
        std::cout << "File .ini not found, quitting" << std::endl;
        return;
    }
    
    m_sensorName = _sensor->GetScopedName();
    //Insert the pointer in the singleton handler for retriving it in the yarp driver
    GazeboYarpPlugins::Handler::getHandler()->setSensor(_sensor.get());
    
    m_parameters.put(YarpIMUScopedName.c_str(), m_sensorName.c_str());
    
    //Open the driver
    if (m_imuDriver.open(m_parameters)) {
        std::cout << "Loaded GazeboYarpIMU Plugin correctly" << std::endl;
    } else {
        std::cout << "GazeboYarpIMU Plugin Load failed: error in opening yarp driver" << std::endl;
    }
    
    std::cout << "GazeboYarpIMU original parameters" << std::endl;
    std::cout << m_parameters.toString() << std::endl;
    std::cout << "GazeboYarpIMU getOptions" << std::endl;
    std::cout << m_imuDriver.getOptions().toString() << std::endl;
}
    
}
