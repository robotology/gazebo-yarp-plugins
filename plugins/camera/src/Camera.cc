/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "Camera.hh"
#include "CameraDriver.h"

#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/log.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <gazebo/sensors/CameraSensor.hh>
#include <yarp/dev/PolyDriver.h>


GZ_REGISTER_SENSOR_PLUGIN(gazebo::GazeboYarpCamera)


namespace gazebo {

GazeboYarpCamera::GazeboYarpCamera() : CameraPlugin(), m_yarp()
{
}

GazeboYarpCamera::~GazeboYarpCamera()
{
    m_cameraDriver.close();
    GazeboYarpPlugins::Handler::getHandler()->removeSensor(m_sensorName);
}

void GazeboYarpCamera::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    if (!m_yarp.checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
        GYPERR << "GazeboYarpCamera::Load error: yarp network does not seem to be available, is the yarpserver running?\n";
        return;
    }

    if (!_sensor) {
        GYPERR << "GazeboYarpCamera plugin requires a CameraSensor.\n";
        return;
    }

    _sensor->SetActive(true);

    // Add my gazebo device driver to the factory.
    ::yarp::dev::Drivers::factory().add(new ::yarp::dev::DriverCreatorOf< ::yarp::dev::GazeboYarpCameraDriver>
                                        ("gazebo_camera", "grabber", "GazeboYarpCameraDriver"));


    //Getting .ini configuration file from sdf
    bool configuration_loaded = GazeboYarpPlugins::loadConfigSensorPlugin(_sensor,_sdf,m_parameters);

    if (!configuration_loaded) {
        return;
    }

    m_sensorName = _sensor->GetScopedName();
    m_sensor = (gazebo::sensors::CameraSensor*)_sensor.get();
    if(m_sensor == NULL)
    {
        GYPERR << "GazeboYarpCamera: m_sensor pointer is NULL" << std::endl;
    }

    // Don't forget to load the camera plugin!!!! ???
//    CameraPlugin::Load(_sensor, _sdf);

    //Insert the pointer in the singleton handler for retriving it in the yarp driver
    GazeboYarpPlugins::Handler::getHandler()->setSensor(_sensor.get());

    m_parameters.put(YarpScopedName.c_str(), m_sensorName.c_str());

    //Open the driver
    bool driverOpenOk = m_cameraDriver.open(m_parameters);

    if (!driverOpenOk) {
        GYPERR << "GazeboYarpCamera Plugin Load failed: error in opening yarp driver\n";
    }

    m_cameraDriver.view(iFrameGrabberImage);
    if(iFrameGrabberImage == NULL)
    {
        GYPERR << "Unable to get the iFrameGrabberImage interface from the device\n";
        return;
    }

}

}
