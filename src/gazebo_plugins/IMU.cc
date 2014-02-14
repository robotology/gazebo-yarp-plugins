/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include <gazebo_yarp_plugins/IMU.hh>
#include <gazebo_yarp_plugins/IMUDriver.h>

#include <gazebo/sensors/ImuSensor.hh>

#include <yarp/dev/ServerInertial.h>
#include <yarp/dev/PolyDriver.h>

#include "gazebo_yarp_plugins/Handler.hh"


using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(GazeboYarpIMU)

#define toDeg(X) (X*180.0/M_PI)

GazeboYarpIMU::GazeboYarpIMU() : SensorPlugin(), _yarp()
{
}

void GazeboYarpIMU::Init()
{
    std::cout<<"*** GazeboYarpIMU plugin started ***"<<std::endl;
    if (!_yarp.checkNetwork())
        std::cout<<"Sorry YARP network does not seem to be available, is the yarp server available?"<<std::endl;
    else
        std::cout<<"YARP Server found!"<<std::endl;
}

GazeboYarpIMU::~GazeboYarpIMU()
{
    std::cout<<"*** GazeboYarpIMU closing ***"<<std::endl;
    _imu_driver.close();
    GazeboYarpPluginHandler::getHandler()->removeSensor(_sensorName);
}

void GazeboYarpIMU::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    
    if (!_sensor)
    {
        gzerr << "GazeboYarpIMU plugin requires a IMUSensor.\n";
        return;
    }

    _sensor->SetActive(true);
    
    // Add my gazebo device driver to the factory.
    yarp::dev::Drivers::factory().add(new yarp::dev::DriverCreatorOf<yarp::dev::GazeboYarpIMUDriver>
                                      ("gazebo_imu", "inertial", "GazeboYarpIMUDriver"));

        
    //Getting .ini configuration file from sdf
    bool configuration_loaded = false;
        
    if(_sdf->HasElement("yarpConfigurationFile") )
    {
        std::string ini_file_name = _sdf->Get<std::string>("yarpConfigurationFile");
        std::string ini_file_path = gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);

        if( ini_file_path != "" && _parameters.fromConfigFile(ini_file_path.c_str()) )
        {
            std::cout << "Found yarpConfigurationFile: loading from " << ini_file_path << std::endl; 
            configuration_loaded = true;
        }
        
    }
        
    if(!configuration_loaded)
    {
        std::cout << "File .ini not found, quitting\n" << std::endl;
        return;
    }
    
    _sensorName = _sensor->GetScopedName();
    //Insert the pointer in the singleton handler for retriving it in the yarp driver
    GazeboYarpPluginHandler::getHandler()->setSensor(_sensor.get());
    
    _parameters.put(yarp_scopedname_parameter.c_str(), _sensorName.c_str());
   
    //Open the driver
    if(_imu_driver.open(_parameters)) {
        std::cout << "Loaded GazeboYarpIMU Plugin correctly" << std::endl;
    } else {
        std::cout << "GazeboYarpIMU Plugin Load failed: error in opening yarp driver" << std::endl;
    }
    
    std::cout << "GazeboYarpIMU original parameters" << std::endl;
    std::cout << _parameters.toString() << std::endl;
    std::cout << "GazeboYarpIMU getOptions" << std::endl;
    std::cout << _imu_driver.getOptions().toString() << std::endl;
}
