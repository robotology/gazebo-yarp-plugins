/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include "gazebo_yarp_plugins/ForceTorque.hh"
#include "gazebo_yarp_plugins/ForceTorqueDriver.h"
#include "gazebo_yarp_plugins/Handler.hh"
#include "gazebo_yarp_plugins/common.h"

#include <gazebo/sensors/ForceTorqueSensor.hh>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>


GZ_REGISTER_SENSOR_PLUGIN(gazebo::GazeboYarpForceTorque)

namespace gazebo {

GazeboYarpForceTorque::GazeboYarpForceTorque() : SensorPlugin(), m_yarp(), m_iWrap(0)
{
}

GazeboYarpForceTorque::~GazeboYarpForceTorque()
{
    std::cout<<"*** GazeboYarpForceTorque closing ***"<<std::endl;
    if(m_iWrap) { m_iWrap->detachAll(); m_iWrap = 0; }
    if( m_forcetorqueWrapper.isValid() ) m_forcetorqueWrapper.close();
    if( m_forceTorqueDriver.isValid() ) m_forceTorqueDriver.close();
    GazeboYarpPlugins::Handler::getHandler()->removeSensor(m_sensorName);
}

void GazeboYarpForceTorque::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    if (!m_yarp.checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
       std::cerr << "GazeboYarpForceTorque::Load error: yarp network does not seem to be available, is the yarpserver running?"<<std::endl;
       return;
    }
    std::cout<<"*** GazeboYarpForceTorque plugin started ***"<<std::endl;
    
    if (!_sensor)
    {
        gzerr << "GazeboYarpForceTorque plugin requires a ForceTorqueSensor.\n";
        return;
    }

    _sensor->SetActive(true);

    // Add my gazebo device driver to the factory.
    ::yarp::dev::Drivers::factory().add(new ::yarp::dev::DriverCreatorOf< ::yarp::dev::GazeboYarpForceTorqueDriver>
                                      ("gazebo_forcetorque", "analogServer", "GazeboYarpForceTorqueDriver"));
        
    //Getting .ini configuration file from sdf
    ::yarp::os::Property wrapper_properties;
    ::yarp::os::Property driver_properties;
    bool configuration_loaded = false;
        
    if(_sdf->HasElement("yarpConfigurationFile") )
    {
        std::string ini_file_name = _sdf->Get<std::string>("yarpConfigurationFile");
        std::string ini_file_path = gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);

        if( ini_file_path != "" && driver_properties.fromConfigFile(ini_file_path.c_str()) )
        {
            std::cout << "Found yarpConfigurationFile: loading from " << ini_file_path << std::endl;
            configuration_loaded = true;
        }
    }
    
    ///< \todo TODO handle in a better way the parameters that are for the wrapper and the one that are for driver
    wrapper_properties = driver_properties;
        
    if( !configuration_loaded )
    {
        std::cout << "File .ini not found, quitting\n" << std::endl;
        return;
    }
    
    m_sensorName = _sensor->GetScopedName();
    //Insert the pointer in the singleton handler for retriving it in the yarp driver
    GazeboYarpPlugins::Handler::getHandler()->setSensor(boost::get_pointer(_sensor));
    
    driver_properties.put(YarpForceTorqueScopedName.c_str(), m_sensorName.c_str());
    
    //Open the wrapper
    //Force the wrapper to be of type "analogServer" (it make sense? probably no)
    wrapper_properties.put("device","analogServer");
    if( m_forcetorqueWrapper.open(wrapper_properties) ) {
        std::cout<<"GazeboYarpForceTorque Plugin: correcly opened GazeboYarpForceTorqueDriver wrapper"<<std::endl;
    } else {
        std::cout<<"GazeboYarpForceTorque Plugin failed: error in opening yarp driver wrapper"<<std::endl;
        return;
    }
   
    //Open the driver
    //Force the device to be of type "gazebo_forcetorque" (it make sense? probably yes)
    driver_properties.put("device","gazebo_forcetorque");
    if( m_forceTorqueDriver.open(driver_properties) ) {
        std::cout<<"GazeboYarpForceTorque Plugin: correcly opened GazeboYarpForceTorqueDriver"<<std::endl;
    } else {
        std::cout<<"GazeboYarpForceTorque Plugin failed: error in opening yarp driver"<<std::endl;
        return;
    }

    //Attach the driver to the wrapper
    ::yarp::dev::PolyDriverList driver_list;
    
    if( !m_forcetorqueWrapper.view(m_iWrap) ) {
        std::cerr << "GazeboYarpForceTorque : error in loading wrapper" << std::endl;
        return;
    }
    
    driver_list.push(&m_forceTorqueDriver,"dummy");
    
    if( m_iWrap->attachAll(driver_list) ) {
        std::cerr << "GazeboYarpForceTorque : wrapper was connected with driver " << std::endl;
    } else {
        std::cerr << "GazeboYarpForceTorque : error in connecting wrapper and device " << std::endl;
    }
    
}

}
