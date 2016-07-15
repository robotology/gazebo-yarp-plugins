/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "MaisSensor.hh"
#include "MaisSensorDriver.h"
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <gazebo/physics/Model.hh>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace std;
namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN(GazeboYarpMaisSensor)

GazeboYarpMaisSensor::GazeboYarpMaisSensor() : m_iWrap(0)
{
  
}

GazeboYarpMaisSensor::~GazeboYarpMaisSensor()
{
    if (m_iWrap)
    {
        m_iWrap->detachAll();
        m_iWrap = 0;
    }
    if (m_wrapper.isValid())
        m_wrapper.close();

    GazeboYarpPlugins::Handler::getHandler()->removeDevice(m_sensorName);
    GazeboYarpPlugins::Handler::getHandler()->removeRobot(m_robotName);

    yarp::os::Network::fini();
}


void GazeboYarpMaisSensor::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    yarp::os::Network::init();

    if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
        yError() << "GazeboYarpMaisSensor::Load error: yarp network does not seem to be available, is the yarpserver running?";
        return;
    }

    if (!_parent) {
        gzerr << "GazeboYarpMaisSensor plugin requires a parent.\n";
        return;
    }

    m_robotName = _parent->GetScopedName();
    GazeboYarpPlugins::Handler::getHandler()->setRobot(get_pointer(_parent));

    // Add the gazebo_controlboard device driver to the factory.
    yarp::dev::Drivers::factory().add(new yarp::dev::DriverCreatorOf<yarp::dev::GazeboYarpMaisSensorDriver>("gazebo_maissensor", "analogServer", "GazeboYarpMaisSensorDriver"));

    //Getting .ini configuration file from sdf
    bool configuration_loaded = false;

    yarp::os::Bottle wrapper_group;
    yarp::os::Bottle driver_group;
    if (_sdf->HasElement("yarpConfigurationFile")) {
        std::string ini_file_name = _sdf->Get<std::string>("yarpConfigurationFile");
        std::string ini_file_path = gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);

        GazeboYarpPlugins::addGazeboEnviromentalVariablesModel(_parent,_sdf,m_parameters);

        bool wipe = false;
        if (ini_file_path != "" && m_parameters.fromConfigFile(ini_file_path.c_str(),wipe))
        {
            yInfo() << "GazeboYarpMaisSensor: Found yarpConfigurationFile: loading from " << ini_file_path;
            m_parameters.put("gazebo_ini_file_path",ini_file_path.c_str());

            wrapper_group = m_parameters.findGroup("WRAPPER");
            if(wrapper_group.isNull())
            {
                yError("GazeboYarpMaisSensor : [WRAPPER] group not found in config file\n");
                return;
            }

            if(m_parameters.check("ROS"))
            {
                yarp::os::ConstString ROS;
                ROS = yarp::os::ConstString ("(") + m_parameters.findGroup("ROS").toString() + yarp::os::ConstString (")");
                wrapper_group.append(yarp::os::Bottle(ROS));
            }
                
            configuration_loaded = true;
        }

    }

    if (!configuration_loaded) {
        yError() << "GazeboYarpMaisSensor: File .ini not found, quitting" ;
        return;
    }
    
    //Open the wrapper
    if( m_wrapper.open(wrapper_group) )
    {
    }
    else
    {
        yError()<<"GazeboYarpMaisSensor Plugin failed: error in opening yarp driver wrapper";
        return;
    };

    
    yarp::os::Bottle *netList = wrapper_group.find("networks").asList();

    if (netList->isNull())
    {
        yError("GazeboYarpControlBoard : net list to attach to was not found, load failed.");
        m_wrapper.close();
        return;
    }
        
    //---------------------------------------------
    yarp::dev::PolyDriverDescriptor newPoly;
    
    if (netList->size()!=1)
    {
        yError("GazeboYarpMaisSensor: size of 'networks' parameter cannot be != 1");
    }
        
    newPoly.key = netList->get(0).asString();
    m_sensorName = m_robotName + "::" + newPoly.key.c_str();
    newPoly.poly = GazeboYarpPlugins::Handler::getHandler()->getDevice(m_sensorName);

    if( newPoly.poly != NULL)
    {
        // device already exists, use it, setting it againg to increment the usage counter.
        yError("mais %s already opened", newPoly.key.c_str());
    }
    else
    {
        driver_group = m_parameters.findGroup(newPoly.key.c_str());
        if (driver_group.isNull())
        {
            yError("GazeboYarpControlBoard::Load  Error: [%s] group not found in config file. Closing wrapper.", newPoly.key.c_str());
            return;
        }

        m_parameters.put("name", newPoly.key.c_str());
        m_parameters.fromString(driver_group.toString(), false);
        m_parameters.put("robotScopedName", m_robotName);
        m_parameters.put("device","gazebo_maissensor");

        newPoly.poly = new yarp::dev::PolyDriver;
        if(! newPoly.poly->open(m_parameters) || ! newPoly.poly->isValid())
        {
            yError() << "mais <" << newPoly.key << "> did not open!!";
            newPoly.poly->close();
            return;
        }
    }
    GazeboYarpPlugins::Handler::getHandler()->setDevice(m_sensorName, newPoly.poly);
    
    //------------------
    if (!m_wrapper.isValid())
    {
        yError("GazeboYarpMaisSensor: wrapper did not open");
    }

    if (!m_wrapper.view(m_iWrap))
    {
        yError("Wrapper interface not found");
    }
 
    //Attach the driver to the wrapper
    yarp::dev::PolyDriverList driver_list;
    
    driver_list.push(newPoly.poly,"dummy");
        
    if( m_iWrap->attachAll(driver_list) ) {
    } else {
        yError() << "GazeboYarpForceTorque : error in connecting wrapper and device ";
    }
}

}
