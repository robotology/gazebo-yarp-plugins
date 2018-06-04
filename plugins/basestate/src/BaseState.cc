/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "BaseState.hh"
#include "BaseStateDriver.h"

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Bottle.h>

GZ_REGISTER_MODEL_PLUGIN(gazebo::GazeboYarpBaseState)

namespace gazebo
{

    GazeboYarpBaseState::GazeboYarpBaseState() : ModelPlugin(),
                                                               m_networkWrapper(0)
    {   

    }
    
    GazeboYarpBaseState::~GazeboYarpBaseState()
    {
        if (m_networkWrapper)
        {
            m_networkWrapper->detachAll();
            m_networkWrapper = nullptr;
        }
        
        if (m_networkDevice.isValid())
        {
            m_networkDevice.close();
        }
        
        if (m_deviceDriver.isValid())
        {
            m_deviceDriver.close();
        }
        
        GazeboYarpPlugins::Handler::getHandler()->removeRobot(m_robot);
        yarp::os::Network::fini();
    }
    
    void GazeboYarpBaseState::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        yarp::os::Network::init();
        if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout))
        {
            yError() << "GazeboYarpBaseState::Load erros: yarp network does not seem to be available, is the yarp server running?";
            return;
        }
        
        if (!_parent)
        {
            yError() << "GazeboYarpBaseState plugin requires a parent \n";
            return;
        }
        
        GazeboYarpPlugins::Handler::getHandler()->setRobot(boost::get_pointer(_parent));
        m_robot = _parent->GetScopedName();
        
        ::yarp::dev::Drivers::factory().add(new ::yarp::dev::DriverCreatorOf< ::yarp::dev::GazeboYarpBaseStateDriver>
                                        ("gazebo_basestate", "analogServer", "GazeboYarpBaseState"));
        
        yarp::os::Bottle networkDeviceProp;
        yarp::os::Bottle deviceDriverProp;
        
        if (_sdf->HasElement("yarpConfigurationFile"))
        {
            std::string ini_file_name = _sdf->Get<std::string>("yarpConfigurationFile");
            std::string ini_file_path = gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);
            
            GazeboYarpPlugins::addGazeboEnviromentalVariablesModel(_parent, _sdf, m_config);
            
            bool wipe = false;
            if (ini_file_path != "" && m_config.fromConfigFile(ini_file_path.c_str(), wipe))
            {
                networkDeviceProp = m_config.findGroup("WRAPPER");
                if (networkDeviceProp.isNull())
                {
                    yError() << "GazeboYarpBaseState plugin failed: [WRAPPER] group not found in config file";
                    return;
                }
                
                deviceDriverProp = m_config.findGroup("DRIVER");
                if (deviceDriverProp.isNull())
                {
                    yError() << "GazeboYarpBaseState plugin failed: [DRIVER] group not found in config file";
                    return;
                }
            }
        }
      
        if (networkDeviceProp.find("device").isNull())
        {
            yError() << "GazeboYarpBaseState plugin failed: Missing parameter \"device\"  under [WRAPPER] in config";
            return;
        }
        
        if (networkDeviceProp.find("device").asString() != "analogServer")
        {
            yError() << "GazeboYarpBaseState plugin failed: \"device\" under [WRAPPER] should be set to \"analogSensor\" network wrapper.";
            return;
        }
        
        if (deviceDriverProp.find("device").isNull())
        {
            yError() << "GazeboYarpBaseState plugin failed: Missing parameter \"device\"  under [DRIVER] in config";
            return;
        }
        
        if (deviceDriverProp.find("device").asString() != "gazebo_basestate")
        {
            yError() << "GazeboYarpBaseState plugin failed: \"device\" under [DRIVER] should be set to \"gazebo_basestate\".";
            return;
        }
        
        if (deviceDriverProp.find("baseLink").isNull())
        {
            yError() << "GazeboYarpBaseState plugin failed: Missing parameter \"baseLink\"  under [DRIVER] in config";
            return;
        }
               
        yarp::os::Bottle& robotConf = deviceDriverProp.addList();
        robotConf.addString("robot");
        robotConf.addString(m_robot.data());
        
        if (deviceDriverProp.find("robot").isNull())
        {
            yError() << "GazeboYarpBaseState plugin failed: robot name not passed to the driver";
            return;
        }
        
        if (!m_networkDevice.open(networkDeviceProp))
        {
            yError() << "GazeboYarpBaseState plugin failed: error opening network wrapper device";
            return;
        }
        
        if (!m_deviceDriver.open(deviceDriverProp))
        {
            yError() << "GazeboYarpBaseState plugin failed: error opening yarp device driver";
            return;
        }
        
        yarp::dev::PolyDriverList driver_list;
        if (!m_networkDevice.view(m_networkWrapper))
        {
            yError() << "GazeboYarpBaseState plugin failed: error in loading wrapper";
            return;
        }
        
        driver_list.push(&m_deviceDriver, "dummy");
        
        if (!m_networkWrapper->attachAll(driver_list))
        {
            yError() << "GazeboYarpBaseState plugin failed: error attaching devices";
            return;
        }
    }
}

