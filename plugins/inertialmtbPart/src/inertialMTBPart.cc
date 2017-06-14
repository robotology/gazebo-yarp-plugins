/*
 * Copyright (C) 2013-2017 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
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
    if(m_iWrap) { m_iWrap->detachAll(); m_iWrap = 0; }
    if(m_inertialMTBwrapper.isValid()) {m_inertialMTBwrapper.close();}
    if(m_inertialMTBpartDriver.isValid()) {m_inertialMTBpartDriver.close();}
    yarp::os::Network::fini();
}

void GazeboYarpInertialMTBPart::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    yarp::os::Network::init();
    if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout))
    {
        yError() << "GazeboYarpInertialMTBPart::Load error: yarp network does not seem to be available, is the yarpserver running?";
        return;
    }

    if (!_parent)
    {
        gzerr << "GazeboYarpInertialMTBPart plugin requires a parent Model." << std::endl;
        return;
    }

    //Get the model scoped name
    m_robotName = _parent->GetScopedName(true);

    //Add my gazebo device driver to the factory.
    ::yarp::dev::Drivers::factory().add(new ::yarp::dev::DriverCreatorOf< ::yarp::dev::GazeboYarpInertialMTBPartDriver>
                                        ("gazebo_inertialMTB", "analogServer", "GazeboYarpInertialMTBPartDriver"));

    //Getting .ini configuration file parameters from sdf
    ::yarp::os::Property plugin_properties;

    bool configuration_loaded = GazeboYarpPlugins::loadConfigModelPlugin(_parent,_sdf,plugin_properties);

    if (!configuration_loaded)
    {
        yError() << "GazeboYarpInertialMTBPart : File .ini not found, load failed." ;
        return;
    }

    /*
     * Open the driver wrapper
     */
    //Retrieve wrapper properties
    ::yarp::os::Bottle wrapper_properties = plugin_properties.findGroup("WRAPPER");
    if(wrapper_properties.isNull())
    {
        yError("GazeboYarpInertialMTBPart : [WRAPPER] group not found in config file\n");
        return;
    }

    //Open the driver wrapper
    if (!m_inertialMTBwrapper.open(wrapper_properties))
    {
        yError() << "GazeboYarpInertialMTBPart Plugin Load failed: error in opening the yarp wrapper";
    }

    /*
     * Open the part driver
     */
    //Retrieve part driver properties
    ::yarp::os::Bottle partDriver_properties = plugin_properties.findGroup("PART_DRIVER");
    if(partDriver_properties.isNull())
    {
        yError("GazeboYarpInertialMTBPart : [PART DRIVER] group not found in config file\n");
        return;
    }

    //Add the model scoped name for later retrieval of the child sensors from the Handler
    yarp::os::Bottle& robotNameProp = partDriver_properties.addList();
    robotNameProp.addString(MTBPartDriverParentScopedName.c_str());
    robotNameProp.addString(m_robotName);

    //Open the part driver
    if (!m_inertialMTBpartDriver.open(partDriver_properties))
    {
        yError() << "GazeboYarpInertialMTBPart Plugin Load failed: error in opening the part driver";
    }

    //Attach the part driver to the wrapper
    m_inertialMTBwrapper.view(m_iWrap);
    ::yarp::dev::PolyDriverList driverList;
    driverList.push(&m_inertialMTBpartDriver,"dummy");
    if(!m_iWrap->attachAll(driverList) )
    {
        yError() << "GazeboYarpForceTorque : error in connecting wrapper and device ";
    }
}

}
