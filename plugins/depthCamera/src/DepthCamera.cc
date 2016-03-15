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
    std::cout << "depth camera LOAD" << std::endl;
    if (!m_yarp.checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
        std::cerr << "GazeboYarpDepthCamera::Load error: yarp network does not seem to be available, is the yarpserver running?"<<std::endl;
        return;
    }


    if (!_sensor) {
        gzerr << "GazeboYarpDepthCamera plugin requires a DepthCameraSensor." << std::endl;
        return;
    }

    _sensor->SetActive(true);

    // Add my gazebo device driver to the factory.
    ::yarp::dev::Drivers::factory().add(new ::yarp::dev::DriverCreatorOf< ::yarp::dev::GazeboYarpDepthCameraDriver>
                                        ("gazebo_depthCamera", "grabber", "GazeboYarpDepthCameraDriver"));


    //Getting .ini configuration file from sdf
    bool configuration_loaded = GazeboYarpPlugins::loadConfigSensorPlugin(_sensor,_sdf,m_parameters);

    if (!configuration_loaded)
    {
        std::cout << "error loading configuration from SDF" << std::endl;
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
        std::cout << "m_sensor == NULL" << std::endl;
    }


    // Don't forget to load the camera plugin!!!! ???
//    DepthCameraPlugin::Load(_sensor, _sdf);

    std::cout << "sensor scoped name is " << m_sensorName.c_str() << std::endl;
    //Insert the pointer in the singleton handler for retriving it in the yarp driver
    GazeboYarpPlugins::Handler::getHandler()->setSensor(_sensor.get());

    m_parameters.put(YarpScopedName.c_str(), m_sensorName.c_str());

    //Open the driver
    if (m_cameraDriver.open(m_parameters)) {
        std::cout << "Loaded GazeboYarpDepthCamera Plugin correctly" << std::endl;
    } else {
        std::cout << "GazeboYarpDepthCamera Plugin Load failed: error in opening yarp driver" << std::endl;
    }

    m_cameraDriver.view(iFrameGrabberImage);
    if(iFrameGrabberImage == NULL)
    {
        std::cout << "Unable to get the iFrameGrabberImage interface from the device" << std::endl;
        return;
    }

}

}
