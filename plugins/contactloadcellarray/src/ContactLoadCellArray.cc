/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "ContactLoadCellArrayDriver.h"
#include "ContactLoadCellArray.hh"

#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>

#include <string>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

GZ_REGISTER_MODEL_PLUGIN(gazebo::GazeboYarpContactLoadCellArray)

namespace gazebo 
{
    GazeboYarpContactLoadCellArray::GazeboYarpContactLoadCellArray() : ModelPlugin(), m_imultwrapper(0)
    {

    }
  

    GazeboYarpContactLoadCellArray::~GazeboYarpContactLoadCellArray()
    {
        if (m_imultwrapper)
        {
            m_imultwrapper->detachAll();
            m_imultwrapper = 0;
        }
    
        if (m_devWrapper.isValid())
        {
            m_devWrapper.close();
        }
    
        if (m_devDriver.isValid())
        {
            m_devDriver.close();
        }
    
        GazeboYarpPlugins::Handler::getHandler()->removeRobot(m_gazeboRobotName);
        yarp::os::Network::fini();
    }


    void GazeboYarpContactLoadCellArray::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        yarp::os::Network::init();
        if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout))
        {
            yError() << "GazeboYarpContactLoadCellArray::Load error: yarp network does not seem to be available, is the yarp server running?";
            return;
        }
    
        if (!_parent)
        {
            gzerr << "GazeboYarpContactLoadCellArray plugin requires a parent \n";
            return;
        }

        // Insert the pointer in the singleton handler for retrieving it in the yarp driver
        GazeboYarpPlugins::Handler::getHandler()->setRobot(boost::get_pointer(_parent));

        m_gazeboRobotName = _parent->GetScopedName();
   
        // Add the gazebo device driver to the factory
        ::yarp::dev::Drivers::factory().add(new ::yarp::dev::DriverCreatorOf< ::yarp::dev::GazeboYarpContactLoadCellArrayDriver>
                                           ("gazebo_contactloadcellarray", "analogServer", "GazeboYarpContactLoadCellArray"));
           
        // Getting .ini config file from _sdf
        yarp::os::Bottle wrapper_properties;
        yarp::os::Bottle driver_properties;
    
        bool configuration_loaded = false;
        if (_sdf->HasElement("yarpConfigurationFile"))
        {
            std::string ini_file_name = _sdf->Get<std::string>("yarpConfigurationFile");
            std::string ini_file_path = gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);
      
            GazeboYarpPlugins::addGazeboEnviromentalVariablesModel(_parent, _sdf, m_parameters);
      
            bool wipe = false;
            if (ini_file_path != "" && m_parameters.fromConfigFile(ini_file_path.c_str(), wipe))
            {
                wrapper_properties = m_parameters.findGroup("WRAPPER");
                if (wrapper_properties.isNull())
                {
                    yError() << "GazeboYarpContactLoadCellArray Plugin failed: [WRAPPER] group not found in config file";
                    return;  
                }

                driver_properties = m_parameters.findGroup("DRIVER");
                if (driver_properties.isNull())
                {
                    yError() << "GazeboYarpContactLoadCellArray Plugin failed: [DRIVER] group not found in config file";
                    return;
                }

                configuration_loaded = true;
            }
        }
    
        if (!configuration_loaded)
        {
            yError() << "GazeboYarpContactLoadCellArray Plugin Failed: File .ini not found, load failed";
            return;
        }
    
        // Check on Required Parameter for Analog Sensor Wrapper
        if (wrapper_properties.find("device").isNull())
        {
            yError() << "GazeboYarpContactLoadCellArray Plugin Failed: Missing parameter \"device\"  under [WRAPPER] in config";
            return;
        }
        m_wrapperName = wrapper_properties.find("device").asString();
        if (m_wrapperName != "analogServer")
        {
            yError() << "GazeboYarpContactLoadCellArray Plugin Failed: \"device\" should be set to \"analogSensor\" network wrapper.";
            return;
        }
    
        // Check on Required Parameter for Device driver
        if (driver_properties.find("device").isNull())
        {
            yError() << "GazeboYarpContactLoadCellArray Plugin Failed: Missing parameter \"device\"  under [DRIVER] in config";
            return;
        }
        m_sensorDriverName = driver_properties.find("device").asString();
        if (m_sensorDriverName != "gazebo_contactloadcellarray")
        {
            yError() << "GazeboYarpContactLoadCellArray Plugin Failed: \"device\" should be set to \"gazebo_contactloadcellarray\".";
            return;
        }
    
        if (driver_properties.find("linkName").isNull())
        {
            yError() << "GazeboYarpContactLoadCellArray Plugin Failed: Missing parameter \"linkName\"  under [DRIVER] in config";
            return;
        }

        // Adding robot name so that it can be used in the driver
        yarp::os::Bottle& driver_robot_config = driver_properties.addList();
        driver_robot_config.addString("robotName");
        driver_robot_config.addString(m_gazeboRobotName.data());
   
        // Check if robot name is added to the pluginParameters
        if (driver_properties.find("robotName").isNull())
        {
            yError() << "GazeboYarpContactLoadCellArray Plugin Failed: Sensor name not passed to the driver";
            return;
        }
    
        // Open the wrapper
        if (!m_devWrapper.open(wrapper_properties))
        {
            yError() << "GazeboYarpContactLoadCellArray Plugin failed: error in opening wrapper for yarp driver";
            return;
        }
    
        // Open the driver
        if (!m_devDriver.open(driver_properties))
        {
            yError() << "GazeboYarpContactLoadCellArray Plugin failed: error in opening yarp driver";
            return;
        }
    
        // Attach the driver to the wrapper
        yarp::dev::PolyDriverList driver_list;
    
        if (!m_devWrapper.view(m_imultwrapper))
        {
            yError() << "GazeboYarpContactLoadCellArray Plugin failed: Error in loading wrapper";
            return;
        }
    
        driver_list.push(&m_devDriver, "dummy");
    
        if (!m_imultwrapper->attachAll(driver_list))
        {
            yError() << "GazeboYarpContactLoadCellArray Plugin failed: Error in connecting the wrapper and device";
        }
     
    } 
} 
