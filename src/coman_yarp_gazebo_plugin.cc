/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia iCub Facility
 * Authors: Lorenzo Natale and Paul Fitzpatrick and Mingo Enrico
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <yarp/os/Network.h>
#include <gazebo_pointer_wrapper.h>
#include <coman.h>

#define ms(X) (X * 1000.0)
#define toRad(X) (X*M_PI/180.0)
#define ROBOT_NAME "COMAN"

namespace gazebo
{
  
  
  /**
   * The gazebo plugin is the "main" of the yarp device,
   * so what it should is to initialize the device, copy the 
   * gazebo pointer, and return
   * 
   * The device will receive the gazebo pointer, parse the model, 
   * and wait for yarp connections and the gazebo wait event.
   */
class coman_yarp_gazebo_plugin : public ModelPlugin
{
public:
    coman_yarp_gazebo_plugin() : _yarp()
    {

    }

    void Init()
    {
        std::cout<<"*** COMAN GAZEBO YARP PLUGIN ***"<<std::endl;
        if (!_yarp.checkNetwork())
            std::cout<<"Sorry YARP network does not seem to be available, is the yarp server available?"<<std::endl;
        else
            std::cout<<"YARP Server found!"<<std::endl;
    }

    ~coman_yarp_gazebo_plugin()
    {
        _driver.close();
        std::cout<<"Goodbye!"<<std::endl;
    }

    /**
     * Saves the gazebo pointer, creates the device driver
     */
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {

      this->_robot = _parent;

        //_dT = ms(this->_robot->GetWorld()->GetPhysicsEngine()->GetUpdatePeriod());
        //std::cout<<"Simulation Time Step: "<<_dT<<" [ms]"<<std::endl;

        //Can this be eliminated??
        gazebo_pointer_wrapper::setModel(this->_robot);
        
        yarp::dev::Drivers::factory().add(new yarp::dev::DriverCreatorOf<yarp::dev::coman>
                                          ("coman", "controlboard", "coman"));
  
        //Getting .ini configuration file from sdf
        bool configuration_loaded = false;
        if(_sdf->HasElement("yarpConfigurationFile") ) {
            std::string ini_file_name = _sdf->Get<std::string>("yarpConfigurationFile");
            std::string ini_file_path = gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);
            yarp::os::Property plugin_parameters;
            if( ini_file_path != "" && plugin_parameters.fromConfigFile(ini_file_path.c_str()) ) {
                std::cout << "Found yarpConfigurationFile: loading from " << ini_file_path << std::endl; 
                _parameters.put("gazebo_ini_file_path",ini_file_path.c_str());
            
                std::string gazebo_group = "GAZEBO";
            
                _parameters.put("device", plugin_parameters.findGroup(gazebo_group.c_str()).find("device").asString().c_str());
                _parameters.put("subdevice", plugin_parameters.findGroup(gazebo_group.c_str()).find("subdevice").asString().c_str());
                _parameters.put("name", plugin_parameters.findGroup(gazebo_group.c_str()).find("name").asString().c_str());

                configuration_loaded = true;
            }
            
        }
        if( !configuration_loaded ) {
            _parameters.put("device", "controlboard");
            _parameters.put("subdevice", "coman");
            _parameters.put("name", "/coman/test");//TODO what's this?
            std::cout << "File .ini not found, loading default parameters" << std::endl;
        }

        _driver.open(_parameters);
        if (!_driver.isValid())
            fprintf(stderr, "Device did not open\n");

        printf("Device initialized correctly, now sitting and waiting cause I am just the main of the yarp device, and the coman is linked to the onUpdate event of gazebo\n");

        std::cout<<"LOADED GAZEBO 2 YARP PLUGIN!"<<std::endl;
    }

private:
    /**
      * Simulation Time Step in ms
      */
    //double _dT;
    physics::ModelPtr _robot;
    yarp::os::Network _yarp;
    yarp::dev::PolyDriver _driver;
    yarp::os::Property _parameters;
   // event::ConnectionPtr updateConnection; // Pointer to the update event connection
};

GZ_REGISTER_MODEL_PLUGIN(coman_yarp_gazebo_plugin)
}
