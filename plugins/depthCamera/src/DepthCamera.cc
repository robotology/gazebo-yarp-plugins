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
    yTrace() << "depth camera LOAD";
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
    bool configuration_loaded = GazeboYarpPlugins::loadConfigSensorPlugin(_sensor,_sdf,m_parameters);

    if (!configuration_loaded)
    {
        yError() << "error loading configuration from SDF";
        return;
    }

#if GAZEBO_MAJOR_VERSION >= 7
    m_sensorName = _sensor->ScopedName();
#else
    m_sensorName = _sensor->GetScopedName();
#endif
    m_sensor = (gazebo::sensors::DepthCameraSensor*)_sensor.get();
    if(m_sensor == NULL)
    {
        yDebug() << "m_sensor == NULL";
    }


    // Don't forget to load the camera plugin!!!! ???
//    DepthCameraPlugin::Load(_sensor, _sdf);

    yDebug() << "GazeboYarpDepthCamera Plugin: sensor scoped name is " << m_sensorName.c_str() ;
    //Insert the pointer in the singleton handler for retriving it in the yarp driver
    GazeboYarpPlugins::Handler::getHandler()->setSensor(_sensor.get());

    m_parameters.put(YarpScopedName.c_str(), m_sensorName.c_str());

    //Open the driver
    if (m_cameraDriver.open(m_parameters)) {
        yInfo() << "Loaded GazeboYarpDepthCamera Plugin correctly";
    } else {
        yError() << "GazeboYarpDepthCamera Plugin Load failed: error in opening yarp driver";
    }

    m_cameraDriver.view(iFrameGrabberImage);
    if(iFrameGrabberImage == NULL)
    {
        yError() << "Unable to get the iFrameGrabberImage interface from the device";
        return;
    }

}

}
