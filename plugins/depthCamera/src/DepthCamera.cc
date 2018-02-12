/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "DepthCamera.hh"
#include "DepthCameraDriver.h"
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <gazebo/sensors/DepthCameraSensor.hh>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>


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
        yError() << "GazeboYarpDepthCamera::Load error: yarp network does not seem to be available, is the yarpserver running?";
        return;
    }


    if (!_sensor) {
        gzerr << "GazeboYarpDepthCamera plugin requires a DepthCameraSensor." << std::endl;
        return;
    }

    _sensor->SetActive(true);

    // Add my gazebo device driver to the factory.
    ::yarp::dev::Drivers::factory().add(new ::yarp::dev::DriverCreatorOf< ::yarp::dev::GazeboYarpDepthCameraDriver>
                                        ("gazebo_depthCamera", "RGBDSensorWrapper", "GazeboYarpDepthCameraDriver"));


    //Getting .ini configuration file from sdf
    bool configuration_loaded = GazeboYarpPlugins::loadConfigSensorPlugin(_sensor,_sdf,m_driverParameters);
    // wrapper params are in the same file along the driver params
    ::yarp::os::Property wrapper_properties = m_driverParameters;

    if (!configuration_loaded)
    {
        yError() << "error loading configuration from SDF";
        return;
    }

    m_sensorName = _sensor->ScopedName();

    m_sensor = (gazebo::sensors::DepthCameraSensor*)_sensor.get();
    if(m_sensor == NULL)
    {
        yDebug() << "m_sensor == NULL";
    }

    //Insert the pointer in the singleton handler for retriving it in the yarp driver
    GazeboYarpPlugins::Handler::getHandler()->setSensor(_sensor.get());

    // Add scoped name to list of params
    m_driverParameters.put(YarpScopedName.c_str(), m_sensorName.c_str());

    ///////////////////////////
    //Open the wrapper, forcing it to be a "RGBDSensorWrapper"
    wrapper_properties.put("device","RGBDSensorWrapper");
    if(wrapper_properties.check("subdevice"))
    {
        yError() << "RGBDSensorWrapper:  Do not use 'subdevice' keyword here since the only supported subdevice is <gazebo_depthCamera>. \
                     Please remove the line 'subdevice " << wrapper_properties.find("subdevice").asString().c_str() << "' from your config file before proceeding";
        return;
    }

    if(!m_cameraWrapper.open(wrapper_properties) )
    {
        yError()<<"GazeboYarpDepthCamera Plugin failed: error in opening yarp wrapper";
        return;
    }

    //Open the driver
    //Force the device to be of type "gazebo_forcetorque" (it make sense? probably yes)
    m_driverParameters.put("device","gazebo_depthCamera");
    yDebug() << "CC: m_driverParameters:\t" << m_driverParameters.toString();

    if(!m_cameraDriver.open(m_driverParameters) )
    {
        yError()<<"GazeboYarpDepthCamera Plugin failed: error in opening yarp driver";
        return;
    }

    //Attach the driver to the wrapper
    ::yarp::dev::PolyDriverList driver_list;

    if(!m_cameraWrapper.view(m_iWrap) )
    {
        yError() << "GazeboYarpDepthCamera : error in loading wrapper";
        return;
    }

    driver_list.push(&m_cameraDriver,"dummy");

    if(!m_iWrap->attachAll(driver_list) )
    {
        yError() << "GazeboYarpDepthCamera : error in connecting wrapper and device ";
    }
}

}
