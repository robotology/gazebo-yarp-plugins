/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "DepthCamera.hh"
#include "DepthCameraDriver.h"
#include "DepthCameraLog.h"
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/LogComponent.h>

#include <gazebo/sensors/DepthCameraSensor.hh>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IMultipleWrapper.h>

using GazeboYarpPlugins::GAZEBODEPTH;

GZ_REGISTER_SENSOR_PLUGIN(gazebo::GazeboYarpDepthCamera)


namespace gazebo {

GazeboYarpDepthCamera::GazeboYarpDepthCamera() : DepthCameraPlugin(), m_yarp()
{
}

GazeboYarpDepthCamera::~GazeboYarpDepthCamera()
{
    m_cameraDriver.close();
    GazeboYarpPlugins::Handler::getHandler()->removeSensor(m_sensorName);
}

void GazeboYarpDepthCamera::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    if (!m_yarp.checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
        yCError(GAZEBODEPTH) << "GazeboYarpDepthCamera::Load error: yarp network does not seem to be available, is the yarpserver running?";
        return;
    }


    if (!_sensor) {
        yCError(GAZEBODEPTH) << "GazeboYarpDepthCamera plugin requires a DepthCameraSensor.";
        return;
    }

    _sensor->SetActive(true);

    #ifndef GAZEBO_YARP_PLUGINS_DISABLE_IMPLICIT_NETWORK_WRAPPERS
    ::yarp::dev::Drivers::factory().add(new ::yarp::dev::DriverCreatorOf< ::yarp::dev::GazeboYarpDepthCameraDriver>
                                                ("gazebo_depthCamera", "RGBDSensorWrapper", "GazeboYarpDepthCameraDriver"));
    #else
    ::yarp::dev::Drivers::factory().add(new ::yarp::dev::DriverCreatorOf< ::yarp::dev::GazeboYarpDepthCameraDriver>
                                                ("gazebo_depthCamera", "", "GazeboYarpDepthCameraDriver"));
    #endif



    //Getting .ini configuration file from sdf
    bool configuration_loaded = GazeboYarpPlugins::loadConfigSensorPlugin(_sensor,_sdf,m_driverParameters);

    #ifndef GAZEBO_YARP_PLUGINS_DISABLE_IMPLICIT_NETWORK_WRAPPERS
    // wrapper params are in the same file along the driver params
    ::yarp::os::Property wrapper_properties = m_driverParameters;
    #endif

    if (!configuration_loaded)
    {
        yCError(GAZEBODEPTH) << "error loading configuration from SDF";
        return;
    }

    m_sensorName = _sensor->ScopedName();

    m_sensor = (gazebo::sensors::DepthCameraSensor*)_sensor.get();
    if(m_sensor == NULL)
    {
        yCDebug(GAZEBODEPTH) << "m_sensor == NULL";
    }

    //Insert the pointer in the singleton handler for retriving it in the yarp driver
    GazeboYarpPlugins::Handler::getHandler()->setSensor(_sensor.get());

    // Add scoped name to list of params
    m_driverParameters.put(YarpScopedName.c_str(), m_sensorName.c_str());

    #ifndef GAZEBO_YARP_PLUGINS_DISABLE_IMPLICIT_NETWORK_WRAPPERS
    bool disable_wrapper = m_driverParameters.check("disableImplicitNetworkWrapper");
    if (!disable_wrapper)
    {
        ///////////////////////////
        //Open the wrapper, forcing it to be a "RGBDSensorWrapper"
        wrapper_properties.put("device","RGBDSensorWrapper");
        if(wrapper_properties.check("subdevice"))
        {
            yCError(GAZEBODEPTH) << "RGBDSensorWrapper:  Do not use 'subdevice' keyword here since the only supported subdevice is <gazebo_depthCamera>. \
                     Please remove the line 'subdevice " << wrapper_properties.find("subdevice").asString().c_str() << "' from your config file before proceeding";
            return;
        }

        if(!m_cameraWrapper.open(wrapper_properties) )
        {
            yCError(GAZEBODEPTH)<<"GazeboYarpDepthCamera Plugin failed: error in opening yarp wrapper";
            return;
        }    
    }
    if (disable_wrapper && !m_driverParameters.check("yarpDeviceName"))
    {
        yCError(GAZEBODEPTH) << "GazeboYarpDepthCamera : missing yarpDeviceName parameter for device" << m_sensorName;
        return;
    }
    #endif

    //Open the driver
    //Force the device to be of type "gazebo_depthCamera" (it make sense? probably yes)
    m_driverParameters.put("device","gazebo_depthCamera");
    yCDebug(GAZEBODEPTH) << "CC: m_driverParameters:\t" << m_driverParameters.toString();

    if(!m_cameraDriver.open(m_driverParameters) )
    {
        yCError(GAZEBODEPTH)<<"GazeboYarpDepthCamera Plugin failed: error in opening yarp driver";
        return;
    }

    #ifndef GAZEBO_YARP_PLUGINS_DISABLE_IMPLICIT_NETWORK_WRAPPERS
    ::yarp::dev::PolyDriverList driver_list;
    if (!disable_wrapper) 
    {
        //Attach the driver to the wrapper

        if(!m_cameraWrapper.view(m_iWrap) )
        {
            yCError(GAZEBODEPTH) << "GazeboYarpDepthCamera : error in loading wrapper";
            return;
        }

        driver_list.push(&m_cameraDriver, "depthcamera");

        if(!m_iWrap->attachAll(driver_list) )
        {
            yCError(GAZEBODEPTH) << "GazeboYarpDepthCamera : error in connecting wrapper and device ";
        }
    }
    #endif
    
    //Register the device with the given name
    std::string scopedDeviceName;

    #ifndef GAZEBO_YARP_PLUGINS_DISABLE_IMPLICIT_NETWORK_WRAPPERS
    if(!m_driverParameters.check("yarpDeviceName"))
    {
        scopedDeviceName = m_sensorName + "::" + driver_list[0]->key;
    }
    else
    {
        scopedDeviceName = m_sensorName + "::" + m_driverParameters.find("yarpDeviceName").asString();
    }
    #else
    if(!m_driverParameters.check("yarpDeviceName"))
    {
        yCError(GAZEBODEPTH) << "missing yarpDeviceName parameter";
        return;
    }
    scopedDeviceName = m_sensorName + "::" + m_driverParameters.find("yarpDeviceName").asString();
    #endif

    if(!GazeboYarpPlugins::Handler::getHandler()->setDevice(scopedDeviceName, &m_cameraDriver))
    {
        yCError(GAZEBODEPTH)<<"GazeboYarpDepthCamera: failed setting scopedDeviceName(=" << scopedDeviceName << ")";
        return;
    }
    yCInfo(GAZEBODEPTH) << "Registered YARP device with instance name:" << scopedDeviceName;
}

} // namespace gazebo
