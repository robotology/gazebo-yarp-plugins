/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia - iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "DoubleLaser.hh"
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <gazebo/physics/Model.hh>

#include <yarp/dev/IMultipleWrapper.h>

#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/LogComponent.h>


/**
 * @file DoubleLaser.cc
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

using namespace std;

namespace {
    YARP_LOG_COMPONENT(GAZEBODOUBLELASER, "gazebo-yarp-plugins.plugins.GazeboYarpDoubleLaser")
}

namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN(GazeboYarpDoubleLaser)

    GazeboYarpDoubleLaser::GazeboYarpDoubleLaser() : m_iWrap_rangeFinder(0), m_iWrap_doublelaser(0)
    {}

    GazeboYarpDoubleLaser::~GazeboYarpDoubleLaser()
    {
        if (m_iWrap_doublelaser) 
        {
            m_iWrap_doublelaser->detachAll();
            m_iWrap_rangeFinder = 0;
        }

        if (m_iWrap_rangeFinder) 
        {
            m_iWrap_rangeFinder->detachAll();
            m_iWrap_rangeFinder = 0;
        }

        if (m_wrapper_rangeFinder.isValid()) 
        {
            m_wrapper_rangeFinder.close();
        }


        for (int n = 0; n < m_lasers.size(); n++) {
            std::string scopedDeviceName = m_sensorName + "::" + m_lasers[n]->key.c_str();
            GazeboYarpPlugins::Handler::getHandler()->removeDevice(scopedDeviceName);
        }

        GazeboYarpPlugins::Handler::getHandler()->removeSensor(m_sensorName);
        yarp::os::Network::fini();
    }


    bool GazeboYarpDoubleLaser::readConfigurationFromFile(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        if (!_sdf->HasElement("yarpConfigurationFile"))
        {
            yCError(GAZEBODOUBLELASER) << "error: unable to load configuration";
            return false;
        }

        std::string ini_file_name = _sdf->Get<std::string>("yarpConfigurationFile");
        std::string ini_file_path = gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);

        if (ini_file_path == "")
        {
            yCError(GAZEBODOUBLELASER) << "ini file path is empty";
            return false;
        }

        GazeboYarpPlugins::addGazeboEnviromentalVariablesModel(_parent,_sdf,m_parameters);

        bool wipe = false; //in order to not clear m_parameters
        if (! m_parameters.fromConfigFile(ini_file_path.c_str(),wipe))
        {
            yCError(GAZEBODOUBLELASER)  << "error reading parameters from config file= " << ini_file_name << "in" <<ini_file_path ;
            return false;
        }

        m_parameters.put("gazebo_ini_file_path",ini_file_path.c_str());

        return true;
    }



    /**
     * Saves the gazebo pointer, creates the device driver
     */
    void GazeboYarpDoubleLaser::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // 1) check params
        if (!_parent)
        {
            yCError(GAZEBODOUBLELASER) << "plugin requires a parent.\n";
            return;
        }

        //2) check yarp network
        yarp::os::Network::init();

        if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout))
        {
            yCError(GAZEBODOUBLELASER) << "yarp network does not seem to be available, is the yarpserver running?";
            return;
        }

        m_sensorName = _parent->GetScopedName();

        //3) load configuration from sdf
        if(!readConfigurationFromFile( _parent, _sdf))
            return;

        // 4) Insert the pointer in the singleton handler for retrieving it in the yarp driver
        GazeboYarpPlugins::Handler::getHandler()->setRobot(get_pointer(_parent));


        //5) open wrapper Rangefinder2DWrapper
        yarp::os::Property wrapper_parameters;
        if(!m_parameters.check("WRAPPER"))
        {
            yCError(GAZEBODOUBLELASER) << "[WRAPPER] group is missing in configuration file";
            return;
        }

        wrapper_parameters.fromString(m_parameters.findGroup("WRAPPER").toString());
        if(m_parameters.check("ROS"))
        {
            wrapper_parameters.addGroup("ROS").fromString(m_parameters.findGroup("ROS").toString());
        }
        else
        {
            yCInfo(GAZEBODOUBLELASER) << "ROS group is missing in configuration file";
        }

        if(! m_wrapper_rangeFinder.open(wrapper_parameters))
        {
            yCError(GAZEBODOUBLELASER) << "error opening yarp driver wrapper Rangefinder2DWrapper";
            return;
        }

        if (!m_wrapper_rangeFinder.view(m_iWrap_rangeFinder)) 
        {
            yCError(GAZEBODOUBLELASER) << "wrapper (Rangefinder2DWrapper) interface not found, load failed.";
            return;
        }

        // 6) Open the driver DoubleLaser
        yarp::os::Property doublelaser_dev_parameters;
        if(!m_parameters.check("onSimulator"))
        {
            yCError(GAZEBODOUBLELASER) << "onSimulator parameter is missing in configuration file";
            return;
        }
        doublelaser_dev_parameters.put("onSimulator", m_parameters.find("onSimulator").asBool());

        doublelaser_dev_parameters.put("device", "cerDoubleLidar");

        if(!m_parameters.check("LASERFRONT-CFG"))
        {
            yCError(GAZEBODOUBLELASER) << "LASERFRONT-CFG group is missing in configuration file";
            return;
        }
        doublelaser_dev_parameters.addGroup("LASERFRONT-CFG").fromString(m_parameters.findGroup("LASERFRONT-CFG").toString());

        if(!m_parameters.check("LASERBACK-CFG"))
        {
            yCError(GAZEBODOUBLELASER) << "LASERBACK-CFG group is missing in configuration file";
            return;
        }
        doublelaser_dev_parameters.addGroup("LASERBACK-CFG").fromString(m_parameters.findGroup("LASERBACK-CFG").toString());
        
        if(m_parameters.check("SENSOR")) {doublelaser_dev_parameters.addGroup("SENSOR").fromString(m_parameters.findGroup("SENSOR").toString());}
        if(m_parameters.check("SKIP"))   {doublelaser_dev_parameters.addGroup("SKIP").fromString(m_parameters.findGroup("SKIP").toString());}
                
        if(!m_driver_doublelaser.open(doublelaser_dev_parameters) )
        {
            yCError(GAZEBODOUBLELASER)<<"error opening DoubleLaser yarp device ";
            return;
        }

        // 7 )finds device of laser front and laser back. the device names are written in configuration .ini file
        yarp::os::Bottle &front_name = m_parameters.findGroup("LASERFRONT-CFG").findGroup("sensorName");
        if(front_name.isNull())
        {
            yCError(GAZEBODOUBLELASER) << "cannot find LASERFRONT-CFG.sensorName parameter";
            return;
        }
         yarp::os::Bottle &back_name = m_parameters.findGroup("LASERBACK-CFG").findGroup("sensorName");
        if(back_name.isNull())
        {
            yCError(GAZEBODOUBLELASER) << "cannot find LASERBACK-CFG.sensorName parameter";
            return;
        }


        std::string laserFront_name = front_name.find("sensorName").asString();
        std::string laserBack_name = back_name.find("sensorName").asString();

        m_driver_laserFront = GazeboYarpPlugins::Handler::getHandler()->getDevice(laserFront_name);
        if(m_driver_laserFront == nullptr)
        {
            yCError(GAZEBODOUBLELASER) << "cannot find laserFront device" << laserFront_name;
            return;
        }

        m_driver_laserBack = GazeboYarpPlugins::Handler::getHandler()->getDevice(laserBack_name);
        if(m_driver_laserBack == nullptr)
        {
            yCError(GAZEBODOUBLELASER) << "cannot find laserBack device" << laserBack_name;
            return;
        }

        yarp::dev::PolyDriverList listoflasers; //it will contain front and back laser device pointers
        yarp::dev::PolyDriverDescriptor laserFront_desc;
        yarp::dev::PolyDriverDescriptor laserBack_desc;

        laserFront_desc.poly = m_driver_laserFront;
        laserFront_desc.key = laserFront_name;
        listoflasers.push(laserFront_desc);

        laserBack_desc.poly = m_driver_laserBack;
        laserBack_desc.key = laserBack_name;
        listoflasers.push(laserBack_desc);

        m_driver_doublelaser.view(m_iWrap_doublelaser);
        if(!m_iWrap_doublelaser->attachAll(listoflasers))
        {
            yCError(GAZEBODOUBLELASER) << "error attaching double laser to front and back laser devices";
            m_driver_doublelaser.close();
            return;
        }


        // 8 )attach rangefinder wrapper to double laser
        yarp::dev::PolyDriverList listofdoubellaser; //it will contain only double laser
        yarp::dev::PolyDriverDescriptor doublelaser_desc;
        doublelaser_desc.poly = &m_driver_doublelaser;
        doublelaser_desc.key = "doublelaser";

        listofdoubellaser.push(doublelaser_desc);

        if(!m_iWrap_rangeFinder->attachAll(listofdoubellaser))
        {
            yCError(GAZEBODOUBLELASER) << "error attaching wrapper to double laser device";
            return;
        }


        //Insert the pointer in the singleton handler for retrieving it in the yarp driver
        GazeboYarpPlugins::Handler::getHandler()->setRobot(get_pointer(_parent));
        
        // 9) Register the device with the given name
        std::string robotName = _parent->GetScopedName();
        std::string scopedDeviceName;
        if(!m_parameters.check("yarpDeviceName"))
        {
            yCError(GAZEBODOUBLELASER)<<"failed getting yarpDeviceName parameter value";
            return;
        }
        else
        {
            scopedDeviceName = robotName + "::" + m_parameters.find("yarpDeviceName").asString();
        }

        if(!GazeboYarpPlugins::Handler::getHandler()->setDevice(scopedDeviceName, &m_driver_doublelaser))
        {
            yCError(GAZEBODOUBLELASER)<<"failed setting scopedDeviceName(=" << scopedDeviceName << ")";
            return;
        }
        yCInfo(GAZEBODOUBLELASER) << "Registered YARP device with instance name:" << scopedDeviceName;
    }
} // namespace gazebo
