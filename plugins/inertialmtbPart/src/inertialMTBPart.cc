/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "inertialMTBPart.hh"
#include "inertialMTBPartDriver.h"

#include <gazebo/physics/physics.hh>
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

GZ_REGISTER_MODEL_PLUGIN(gazebo::GazeboYarpInertialMTBPart)

namespace gazebo {

GazeboYarpInertialMTBPart::GazeboYarpInertialMTBPart() : ModelPlugin()
{
}

GazeboYarpInertialMTBPart::~GazeboYarpInertialMTBPart()
{
    m_wrapper.close();
    yarp::os::Network::fini();
}

void GazeboYarpInertialMTBPart::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    yarp::os::Network::init();
    if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
        yError() << "GazeboYarpInertialMTBPart::Load error: yarp network does not seem to be available, is the yarpserver running?";
        return;
    }

    if (!_parent) {
        gzerr << "GazeboYarpInertialMTBPart plugin requires a parent Model." << std::endl;
        return;
    }

    // Get the model scoped name
    m_robotName = _parent->GetScopedName();

    // Add my gazebo device driver to the factory.
    ::yarp::dev::Drivers::factory().add(new ::yarp::dev::DriverCreatorOf< ::yarp::dev::GazeboYarpInertialMTBPartDriver>
                                        ("gazebo_inertialMTB", "analogServer", "GazeboYarpInertialMTBPartDriver"));

    //Getting .ini configuration file parameters from sdf
    ::yarp::os::Property plugin_properties;
    bool configuration_loaded = GazeboYarpPlugins::loadConfigModelPlugin(_parent,_sdf,plugin_properties);

    if (!configuration_loaded) {
        yError() << "GazeboYarpInertialMTBPart : File .ini not found, load failed." ;
        return;
    }

    //Add the model scoped name for later retrieval of the child sensors from the Handler
    plugin_properties.put(MTBPartDriverParentScopedName.c_str(), m_robotName);

    //Open the driver wrapper and the driver
    if (m_wrapper.open(plugin_properties)) {
    } else {
        yError() << "GazeboYarpInertialMTBPart Plugin Load failed: error in opening yarp driver";
    }
}

}
