/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include <gazebo_yarp_plugins/ForceTorque.hh>
#include <gazebo_yarp_plugins/ForceTorqueDriver.h>

#include <yarp/dev/PolyDriver.h>

#include "gazebo_yarp_plugins/Handler.hh"


using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(GazeboYarpForceTorque)

#define toDeg(X) (X*180.0/M_PI)

GazeboYarpForceTorque::GazeboYarpForceTorque() : SensorPlugin(), _yarp()
{
}

void GazeboYarpForceTorque::Init()
{
    std::cout<<"*** GazeboYarpForceTorque plugin started ***"<<std::endl;
    if (!_yarp.checkNetwork())
        std::cout<<"Sorry YARP network does not seem to be available, is the yarp server available?"<<std::endl;
    else
        std::cout<<"YARP Server found!"<<std::endl;
}

GazeboYarpForceTorque::~GazeboYarpForceTorque()
{
    std::cout<<"*** GazeboYarpForceTorque closing ***"<<std::endl;
}

void GazeboYarpForceTorque::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    
    if (!_sensor)
    {
        gzerr << "GazeboYarpForceTorque plugin requires a ForceTorqueSensor.\n";
        return;
    }

    _sensor->SetActive(true);
    
  

    // Add my gazebo device driver to the factory.
    yarp::dev::Drivers::factory().add(new yarp::dev::DriverCreatorOf<yarp::dev::GazeboYarpForceTorqueDriver>
                                      ("gazebo_forcetorque", "analogServer", "GazeboYarpForceTorqueDriver"));

        
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
        
    if( !configuration_loaded )
    {
        std::cout << "File .ini not found, quitting\n" << std::endl;
        return;
    }
    
    //Insert the pointer in the singleton handler for retriving it in the yarp driver
    GazeboYarpPluginHandler::getHandler()->setSensor(boost::get_pointer(_sensor), _sdf);
    
    _parameters.put(yarp_scopedname_parameter.c_str(),_sensor->GetScopedName().c_str());
   
    //Open the driver
    _forcetorque_driver.open(_parameters);

    std::cout<<"Loaded GazeboYarpForceTorque Plugin correctly"<<std::endl;
}
