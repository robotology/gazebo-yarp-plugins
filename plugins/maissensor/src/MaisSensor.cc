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
    if (m_iWrap) {
        m_iWrap->detachAll();
        m_iWrap = 0;
    }
    if (m_wrapper.isValid())
        m_wrapper.close();

    for (int n = 0; n < m_controlBoards.size(); n++)
    {
        std::string scopedDeviceName = m_robotName + "::" + m_controlBoards[n]->key.c_str();
        GazeboYarpPlugins::Handler::getHandler()->removeDevice(scopedDeviceName);
    }

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

            configuration_loaded = true;
        }

    }

    if (!configuration_loaded) {
        yError() << "GazeboYarpMaisSensor: File .ini not found, quitting" ;
        return;
    }
           
    //Open the wrapper
    //Force the wrapper to be of type "analogServer" (it make sense? probably no)
    yarp::os::Property wrapper_properties = m_parameters;
    wrapper_properties.put("device","analogServer");
    if( m_wrapper.open(wrapper_properties) ) {
    } else {
        yError()<<"GazeboYarpForceTorque Plugin failed: error in opening yarp driver wrapper";
        return;
    };

                //---------------------------------------------
            yarp::dev::PolyDriverDescriptor newPoly;

            newPoly.key = "left_hand_mais";
            std::string scopedDeviceName = m_robotName + "::" + newPoly.key.c_str();
            newPoly.poly = GazeboYarpPlugins::Handler::getHandler()->getDevice(scopedDeviceName);

            if( newPoly.poly != NULL)
            {
                // device already exists, use it, setting it againg to increment the usage counter.
                yWarning("mais %s already opened", newPoly.key.c_str());

            }
            else
            {
                driver_group = m_parameters.findGroup(newPoly.key.c_str());
                if (driver_group.isNull()) {
                    yError("GazeboYarpControlBoard::Load  Error: [%s] group not found in config file. Closing wrapper.", newPoly.key.c_str());
                    return;
                }

                m_parameters.put("name", newPoly.key.c_str());
                m_parameters.fromString(driver_group.toString(), false);
                m_parameters.put("robotScopedName", m_robotName);

                 newPoly.poly = new yarp::dev::PolyDriver;
                if(! newPoly.poly->open(m_parameters) || ! newPoly.poly->isValid())
                {
                    yError() << "mais <" << newPoly.key << "> did not open!!";
                    for(int idx=0; idx<m_controlBoards.size(); idx++)
                    {
                        m_controlBoards[idx]->poly->close();
                    }
                    return;
                }
            }
            GazeboYarpPlugins::Handler::getHandler()->setDevice(scopedDeviceName, newPoly.poly);
            m_controlBoards.push(newPoly);
            //------------------
            
    if (!m_wrapper.isValid())
        yError("GazeboYarpMaisSensor: wrapper did not open");

    if (!m_wrapper.view(m_iWrap)) {
        yError("Wrapper interface not found");
    }
    
    yDebug() << ">>>>>>>Load";
/*
        //Attach the driver to the wrapper
    yarp::dev::PolyDriverList driver_list;
    
    driver_list.push(&m_maisSensorDriver,"dummy");
        
    if( m_iWrap->attachAll(driver_list) ) {
    } else {
        yError() << "GazeboYarpForceTorque : error in connecting wrapper and device ";
    }*/
}

}
