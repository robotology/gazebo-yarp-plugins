/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */
#include <GazeboYarpControlBoard.hh>

namespace gazebo
{
    
GZ_REGISTER_MODEL_PLUGIN(GazeboYarpControlBoard)

    GazeboYarpControlBoard::GazeboYarpControlBoard() : _yarp()
    {

    }

    void GazeboYarpControlBoard::Init()
    {
        std::cout<<"*** COMAN GAZEBO YARP PLUGIN ***"<<std::endl;
        if (!_yarp.checkNetwork())
            std::cout<<"Sorry YARP network does not seem to be available, is the yarp server available?"<<std::endl;
        else
            std::cout<<"YARP Server found!"<<std::endl;
    }

    GazeboYarpControlBoard::~GazeboYarpControlBoard()
    {
        _driver.close();
        std::cout<<"Goodbye!"<<std::endl;
    }

    /**
     * Saves the gazebo pointer, creates the device driver
     */
    void GazeboYarpControlBoard::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {

      this->_robot = _parent;

        yarp::dev::Drivers::factory().add(new yarp::dev::DriverCreatorOf<yarp::dev::GazeboYarpControlBoardDriver>
                                          ("gazebo_controlboard", "controlboard", "GazeboYarpControlBoardDriver"));
  
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
            _parameters.put("subdevice", "gazebo_controlboard");
            _parameters.put("name", "/coman/test");//TODO what's this?
            std::cout << "File .ini not found, loading default parameters" << std::endl;
        }

        //Now I love everything and every interface
        std::ostringstream archive_stream;
        boost::archive::text_oarchive archive(archive_stream);
        uintptr_t cast_boost_to_pointer=(uintptr_t)_parent.get();
        archive<<cast_boost_to_pointer;
        _parameters.put("loving_gazebo_pointer",archive_stream.str().c_str());

        _driver.open(_parameters);
    
        if (!_driver.isValid())
            fprintf(stderr, "Device did not open\n");

        printf("Device initialized correctly, now sitting and waiting cause I am just the main of the yarp device, and the coman is linked to the onUpdate event of gazebo\n");

        std::cout<<"Loaded GazeboYarpControlBoard Plugin"<<std::endl;
    }


}
