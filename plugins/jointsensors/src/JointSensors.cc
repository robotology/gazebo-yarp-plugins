/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "JointSensorsDriver.h"
#include "JointSensors.hh"
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>


#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

GZ_REGISTER_MODEL_PLUGIN(gazebo::GazeboYarpJointSensors)

namespace gazebo {

GazeboYarpJointSensors::GazeboYarpJointSensors() : ModelPlugin(), m_iWrap(0)
{
}

GazeboYarpJointSensors::~GazeboYarpJointSensors()
{
    if (m_iWrap) {
        m_iWrap->detachAll();
        m_iWrap = 0;
    }
    if (m_jointsensorsWrapper.isValid())
        m_jointsensorsWrapper.close();
    if (m_jointsensorsDriver.isValid())
        m_jointsensorsDriver.close();
    GazeboYarpPlugins::Handler::getHandler()->removeRobot(m_robotName);
    yarp::os::Network::fini();
}

void GazeboYarpJointSensors::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    yarp::os::Network::init();
    if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
       yError() << "GazeboYarpJointSensors::Load error: yarp network does not seem to be available, is the yarpserver running?";
       return;
    }

    if (!_parent) {
        gzerr << "GazeboYarpJointSensors plugin requires a parent.\n";
        return;
    }

    // Add my gazebo device driver to the factory.
    ::yarp::dev::Drivers::factory().add(new ::yarp::dev::DriverCreatorOf< ::yarp::dev::GazeboYarpJointSensorsDriver>
                                      ("gazebo_jointsensors", "analogServer", "GazeboYarpJointSensorsDriver"));

    //Getting .ini configuration file from sdf
    ::yarp::os::Property wrapper_properties;
    ::yarp::os::Property driver_properties;

    bool configuration_loaded = GazeboYarpPlugins::loadConfigModelPlugin(_parent,_sdf,driver_properties);

    if (!configuration_loaded) {
        return;
    };

    ///< \todo TODO handle in a better way the parameters that are for the wrapper and the one that are for driver
    wrapper_properties = driver_properties;


    m_robotName = _parent->GetScopedName();
    //Insert the pointer in the singleton handler for retriving it in the yarp driver
    GazeboYarpPlugins::Handler::getHandler()->setRobot(boost::get_pointer(_parent));

    driver_properties.put("robotScopedName", m_robotName.c_str());

    //Open the wrapper
    //Force the wrapper to be of type "analogServer" (it make sense? probably no)
    wrapper_properties.put("device","analogServer");
    if (!m_jointsensorsWrapper.open(wrapper_properties)) {
        yError() << "GazeboYarpJointSensors Plugin failed: error in opening yarp driver wrapper";
        return;
    }

    //Open the driver
    //Force the device to be of type "gazebo_jointsensors" (it make sense? probably yes)
    driver_properties.put("device","gazebo_jointsensors");
    if (!m_jointsensorsDriver.open(driver_properties)) {
        yError() << "GazeboYarpJointSensors Plugin failed: error in opening yarp driver";
        return;
    }

    //Attach the driver to the wrapper
    ::yarp::dev::PolyDriverList driver_list;

    if (!m_jointsensorsWrapper.view(m_iWrap)) {
        yError() << "GazeboYarpJointSensors : error in loading wrapper";
        return;
    }

    driver_list.push(&m_jointsensorsDriver, "dummy");

    if (m_iWrap->attachAll(driver_list)) {
    } else {
        yError() << "GazeboYarpJointSensors : error in connecting wrapper and device ";
    }

}

}
