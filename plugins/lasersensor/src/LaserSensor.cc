/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "LaserSensor.hh"
#include "LaserSensorDriver.h"
#include "LaserSensorLog.h"
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <gazebo/sensors/RaySensor.hh>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IMultipleWrapper.h>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>

using GazeboYarpPlugins::GAZEBOLASER;

GZ_REGISTER_SENSOR_PLUGIN(gazebo::GazeboYarpLaserSensor)

namespace gazebo {

GazeboYarpLaserSensor::GazeboYarpLaserSensor() :
#ifndef GAZEBO_YARP_PLUGINS_DISABLE_IMPLICIT_NETWORK_WRAPPERS
SensorPlugin(),
m_iWrap(0)
#else
SensorPlugin()
#endif
{
}

GazeboYarpLaserSensor::~GazeboYarpLaserSensor()
{
    #ifndef GAZEBO_YARP_PLUGINS_DISABLE_IMPLICIT_NETWORK_WRAPPERS
    if(m_iWrap) { m_iWrap->detachAll(); m_iWrap = 0; }
    if( m_laserWrapper.isValid() ) m_laserWrapper.close();
    #endif

    if( m_laserDriver.isValid() ) m_laserDriver.close();
    GazeboYarpPlugins::Handler::getHandler()->removeSensor(m_sensorName);
    yarp::os::Network::fini();
}

void GazeboYarpLaserSensor::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    yarp::os::Network::init();
    if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout))
    {
       yCError(GAZEBOLASER) << "Load error: yarp network does not seem to be available, is the yarpserver running?";
       return;
    }

    if (!_sensor)
    {
        yCError(GAZEBOLASER) << "the plugin requires a LaserSensor.\n";
        return;
    }

    _sensor->SetActive(true);

    // Add my gazebo device driver to the factory.
    #ifndef GAZEBO_YARP_PLUGINS_DISABLE_IMPLICIT_NETWORK_WRAPPERS
    ::yarp::dev::Drivers::factory().add(new ::yarp::dev::DriverCreatorOf< ::yarp::dev::GazeboYarpLaserSensorDriver>
                                      ("gazebo_laserSensor", "Rangefinder2DWrapper", "GazeboYarpLaserSensorDriver"));
    ::yarp::os::Property wrapper_properties;
    #else
    ::yarp::dev::Drivers::factory().add(new ::yarp::dev::DriverCreatorOf< ::yarp::dev::GazeboYarpLaserSensorDriver>
                                      ("gazebo_laserSensor", "", "GazeboYarpLaserSensorDriver"));
    #endif

    //Getting .ini configuration file from sdf
    ::yarp::os::Property driver_properties;
    bool configuration_loaded = GazeboYarpPlugins::loadConfigSensorPlugin(_sensor,_sdf,driver_properties);

    if (!configuration_loaded)
    {
        yCError(GAZEBOLASER) << "Load error: unabble to load configuration?";
        return;
    };

    #ifndef GAZEBO_YARP_PLUGINS_DISABLE_IMPLICIT_NETWORK_WRAPPERS
    ///< \todo TODO handle in a better way the parameters that are for the wrapper and the one that are for driver
    wrapper_properties = driver_properties;
    #endif

    m_sensorName = _sensor->ScopedName();

    //Insert the pointer in the singleton handler for retriving it in the yarp driver
    GazeboYarpPlugins::Handler::getHandler()->setSensor(_sensor.get());

    driver_properties.put(YarpLaserSensorScopedName.c_str(), m_sensorName.c_str());

    #ifndef GAZEBO_YARP_PLUGINS_DISABLE_IMPLICIT_NETWORK_WRAPPERS
    //Open the wrapper
    wrapper_properties.put("device","Rangefinder2DWrapper");
    if( m_laserWrapper.open(wrapper_properties) ) {
    } else
    {
        yCError(GAZEBOLASER)<<"Plugin failed: error in opening yarp driver wrapper";
        return;
    }
    #endif

    //Open the driver
    //Force the device to be of type "gazebo_laserSensor" (it make sense? probably yes)
    driver_properties.put("device","gazebo_laserSensor");
    if( m_laserDriver.open(driver_properties) ) {
    } else 
    {
        yCError(GAZEBOLASER)<<"Plugin failed: error in opening yarp driver";
        return;
    }

    #ifndef GAZEBO_YARP_PLUGINS_DISABLE_IMPLICIT_NETWORK_WRAPPERS
    //Attach the driver to the wrapper
    ::yarp::dev::PolyDriverList driver_list;

    if( !m_laserWrapper.view(m_iWrap) )
    {
        yCError(GAZEBOLASER) << "GazeboYarpLaserSensor : error in loading wrapper" ;
        return;
    }

    driver_list.push(&m_laserDriver, "lasersensor");

    if( m_iWrap->attachAll(driver_list) ) {
    } else
    {
        yCError(GAZEBOLASER) << "GazeboYarpLaserSensor : error in connecting wrapper and device " ;
    }
    #endif

    //Register the device with the given name
    std::string sensorName = _sensor->ScopedName();
    std::string scopedDeviceName;
    if(driver_properties.check("deviceId"))
    {
        yCWarning(GAZEBOLASER) << "deviceId parameter has been deprecated. Please use yarpDeviceName instead";
        scopedDeviceName = sensorName + "::" + driver_properties.find("deviceId").asString();
    }
    else if(!driver_properties.check("yarpDeviceName"))
    {
        yCError(GAZEBOLASER)<<"failed getting yarpDeviceName parameter value";
        #ifndef GAZEBO_YARP_PLUGINS_DISABLE_IMPLICIT_NETWORK_WRAPPERS
        scopedDeviceName = sensorName + "::" + driver_list[0]->key;
        #else
        return;
        #endif
    }
    else
    {
        scopedDeviceName = sensorName + "::" + driver_properties.find("yarpDeviceName").asString();
    }


    if(!GazeboYarpPlugins::Handler::getHandler()->setDevice(scopedDeviceName, &m_laserDriver))
    {
        yCError(GAZEBOLASER)<<"failed setting scopedDeviceName(=" << scopedDeviceName << ")";
        return;
    }
    yCInfo(GAZEBOLASER) << "Registered YARP device with instance name:" << scopedDeviceName;
}

} // namespace gazebo
