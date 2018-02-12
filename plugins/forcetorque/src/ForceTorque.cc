/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "ForceTorque.hh"
#include "ForceTorqueDriver.h"
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <gazebo/sensors/ForceTorqueSensor.hh>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>


GZ_REGISTER_SENSOR_PLUGIN(gazebo::GazeboYarpForceTorque)

namespace gazebo {

GazeboYarpForceTorque::GazeboYarpForceTorque() : SensorPlugin(), m_iWrap(0)
{
}

GazeboYarpForceTorque::~GazeboYarpForceTorque()
{
    if(m_iWrap) { m_iWrap->detachAll(); m_iWrap = 0; }
    if( m_forcetorqueWrapper.isValid() ) m_forcetorqueWrapper.close();
    if( m_forceTorqueDriver.isValid() ) m_forceTorqueDriver.close();
    GazeboYarpPlugins::Handler::getHandler()->removeSensor(m_sensorName);
    yarp::os::Network::fini();
}

void GazeboYarpForceTorque::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    yarp::os::Network::init();
    if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
       yError() << "GazeboYarpForceTorque::Load error: yarp network does not seem to be available, is the yarpserver running?";
       return;
    }

    if (!_sensor)
    {
        gzerr << "GazeboYarpForceTorque plugin requires a ForceTorqueSensor.\n";
        return;
    }

    _sensor->SetActive(true);

    // Add my gazebo device driver to the factory.
    ::yarp::dev::Drivers::factory().add(new ::yarp::dev::DriverCreatorOf< ::yarp::dev::GazeboYarpForceTorqueDriver>
                                      ("gazebo_forcetorque", "analogServer", "GazeboYarpForceTorqueDriver"));

    //Getting .ini configuration file from sdf
    ::yarp::os::Property wrapper_properties;
    ::yarp::os::Property driver_properties;
    bool configuration_loaded = GazeboYarpPlugins::loadConfigSensorPlugin(_sensor,_sdf,driver_properties);

    if (!configuration_loaded) {
        return;
    };


    ///< \todo TODO handle in a better way the parameters that are for the wrapper and the one that are for driver
    wrapper_properties = driver_properties;

    if( !configuration_loaded )
    {
        return;
    }
    m_sensorName = _sensor->ScopedName();

    //Insert the pointer in the singleton handler for retriving it in the yarp driver
    GazeboYarpPlugins::Handler::getHandler()->setSensor(_sensor.get());

    driver_properties.put(YarpForceTorqueScopedName.c_str(), m_sensorName.c_str());

    //Open the wrapper
    //Force the wrapper to be of type "analogServer" (it make sense? probably no)
    wrapper_properties.put("device","analogServer");
    if( m_forcetorqueWrapper.open(wrapper_properties) ) {
    } else {
        yError()<<"GazeboYarpForceTorque Plugin failed: error in opening yarp driver wrapper";
        return;
    }

    //Open the driver
    //Force the device to be of type "gazebo_forcetorque" (it make sense? probably yes)
    driver_properties.put("device","gazebo_forcetorque");
    if( m_forceTorqueDriver.open(driver_properties) ) {
    } else {
        yError()<<"GazeboYarpForceTorque Plugin failed: error in opening yarp driver";
        return;
    }

    //Attach the driver to the wrapper
    ::yarp::dev::PolyDriverList driver_list;

    if( !m_forcetorqueWrapper.view(m_iWrap) ) {
        yError() << "GazeboYarpForceTorque : error in loading wrapper";
        return;
    }

    driver_list.push(&m_forceTorqueDriver,"dummy");

    if( m_iWrap->attachAll(driver_list) ) {
    } else {
        yError() << "GazeboYarpForceTorque : error in connecting wrapper and device ";
    }

}

}
