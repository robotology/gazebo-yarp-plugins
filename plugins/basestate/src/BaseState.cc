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
        if (m_deviceRegistered)
        {
            GazeboYarpPlugins::Handler::getHandler()->removeDevice(m_scopedDeviceName);
            m_deviceRegistered = false;
        }

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

        ::yarp::dev::Drivers::factory().add(new ::yarp::dev::DriverCreatorOf<::yarp::dev::GazeboYarpBaseStateDriver>("gazebo_basestate", "", "GazeboYarpBaseState"));

        // Getting .ini configuration file parameters from sdf
        bool configuration_loaded = GazeboYarpPlugins::loadConfigModelPlugin(_parent, _sdf, m_config);

        if (!configuration_loaded)
        {
            yError() << "GazeboYarpBaseState : File .ini not found, load failed." ;
            return;
        }

        bool disable_wrapper = m_config.check("disableImplicitNetworkWrapper");

        // New behaviour to use
        if (disable_wrapper)
        {
            if (m_config.find("yarpDeviceName").isNull())
            {
                yError() << "GazeboYarpBaseState plugin failed: Missing parameter \"yarpDeviceName\"  under in config";
                return;
            }

            if (m_config.find("baseLink").isNull())
            {
                yError() << "GazeboYarpBaseState plugin failed: Missing parameter \"baseLink\"  under in config";
                return;
            }

            m_config.put("device", "gazebo_basestate");
            m_config.put("robot", m_robot);

            if (!m_deviceDriver.open(m_config))
            {
                yError() << "GazeboYarpBaseState plugin failed: error opening yarp device driver";
                return;
            }

            m_scopedDeviceName = m_robot + "::" + m_config.find("yarpDeviceName").asString();

            if (!GazeboYarpPlugins::Handler::getHandler()->setDevice(m_scopedDeviceName, &m_deviceDriver))
            {
                yError() << "GazeboYarpForceTorque: failed setting scopedDeviceName(=" << m_scopedDeviceName << ")";
                return;
            }
            m_deviceRegistered = true;
            yInfo() << "Registered YARP device with instance name:" << m_scopedDeviceName;
        }

        // Legacy behaviour that will be removed
        if (!disable_wrapper)
        {
            yarp::os::Bottle networkDeviceProp = m_config.findGroup("WRAPPER");
            if (networkDeviceProp.isNull())
            {
                yError() << "GazeboYarpBaseState plugin failed: [WRAPPER] group not found in config file";
                return;
            }

            yarp::os::Bottle deviceDriverProp = m_config.findGroup("DRIVER");
            if (deviceDriverProp.isNull())
            {
                yError() << "GazeboYarpBaseState plugin failed: [DRIVER] group not found in config file";
                return;
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

            yarp::os::Bottle &robotConf = deviceDriverProp.addList();
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
}

