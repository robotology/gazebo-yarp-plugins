/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include "gazebo_yarp_plugins/JointSensorsDriver.h"
#include "gazebo_yarp_plugins/JointSensors.hh"
#include "gazebo_yarp_plugins/Handler.hh"
#include "gazebo_yarp_plugins/common.h"

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>


GZ_REGISTER_MODEL_PLUGIN(gazebo::GazeboYarpJointSensors)

namespace gazebo {

GazeboYarpJointSensors::GazeboYarpJointSensors() : ModelPlugin(), _yarp(), _iWrap(0)
{
}

GazeboYarpJointSensors::~GazeboYarpJointSensors()
{
    std::cout<<"*** GazeboYarpJointSensors closing ***"<<std::endl;
    if(_iWrap) { _iWrap->detachAll(); _iWrap = 0; }
    if( _jointsensors_wrapper.isValid() ) _jointsensors_wrapper.close();
    if( _jointsensors_driver.isValid() ) _jointsensors_driver.close();
    GazeboYarpPlugins::Handler::getHandler()->removeRobot(_robotName);
}

void GazeboYarpJointSensors::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    if( !_yarp.checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout) ) {
       std::cerr << "GazeboYarpJointSensors::Load error: yarp network does not seem to be available, is the yarpserver running?"<<std::endl;
       return;
    }
    std::cout<<"*** GazeboYarpJointSensors plugin started ***"<<std::endl;

    if (!_parent)
    {
        gzerr << "GazeboYarpJointSensors plugin requires a parent.\n";
        return;
    }

    // Add my gazebo device driver to the factory.
    ::yarp::dev::Drivers::factory().add(new ::yarp::dev::DriverCreatorOf< ::yarp::dev::GazeboYarpJointSensorsDriver>
                                      ("gazebo_jointsensors", "analogServer", "GazeboYarpJointSensorsDriver"));

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

    _robotName = _parent->GetScopedName();
    //Insert the pointer in the singleton handler for retriving it in the yarp driver
    GazeboYarpPlugins::Handler::getHandler()->setRobot(boost::get_pointer(_parent));

    driver_properties.put("robotScopedName", _robotName.c_str());

    //Open the wrapper
    //Force the wrapper to be of type "analogServer" (it make sense? probably no)
    wrapper_properties.put("device","analogServer");
    if( _jointsensors_wrapper.open(wrapper_properties) ) {
        std::cout<<"GazeboYarpJointSensors Plugin: correcly opened GazeboYarpJointSensorsDriver wrapper"<<std::endl;
    } else {
        std::cout<<"GazeboYarpJointSensors Plugin failed: error in opening yarp driver wrapper"<<std::endl;
        return;
    }

    //Open the driver
    //Force the device to be of type "gazebo_jointsensors" (it make sense? probably yes)
    driver_properties.put("device","gazebo_jointsensors");
    if( _jointsensors_driver.open(driver_properties) ) {
        std::cout<<"GazeboYarpJointSensors Plugin: correcly opened GazeboYarpJointSensorsDriver"<<std::endl;
    } else {
        std::cout<<"GazeboYarpJointSensors Plugin failed: error in opening yarp driver"<<std::endl;
        return;
    }

    //Attach the driver to the wrapper
    ::yarp::dev::PolyDriverList driver_list;

    if( !_jointsensors_wrapper.view(_iWrap) ) {
        std::cerr << "GazeboYarpJointSensors : error in loading wrapper" << std::endl;
        return;
    }

    driver_list.push(&_jointsensors_driver,"dummy");

    if( _iWrap->attachAll(driver_list) ) {
        std::cerr << "GazeboYarpJointSensors : wrapper was connected with driver " << std::endl;
    } else {
        std::cerr << "GazeboYarpJointSensors : error in connecting wrapper and device " << std::endl;
    }

}

}
