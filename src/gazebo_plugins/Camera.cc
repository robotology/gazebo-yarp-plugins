/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include "gazebo_yarp_plugins/Camera.hh"
#include "gazebo_yarp_plugins/CameraDriver.h"
#include "gazebo_yarp_plugins/Handler.hh"
#include "gazebo_yarp_plugins/common.h"

#include <gazebo/sensors/CameraSensor.hh>

#include <yarp/dev/ServerInertial.h>
#include <yarp/dev/PolyDriver.h>


GZ_REGISTER_SENSOR_PLUGIN(gazebo::GazeboYarpCamera)


namespace gazebo {
    
GazeboYarpCamera::GazeboYarpCamera() : CameraPlugin(), m_yarp()
{
    std::cout << "*** GazeboYarpCamera contructor ***" << std::endl;
}

GazeboYarpCamera::~GazeboYarpCamera()
{
    std::cout << "*** GazeboYarpCamera closing ***" << std::endl;
    m_cameraDriver.close();
    GazeboYarpPlugins::Handler::getHandler()->removeSensor(m_sensorName);
}

void GazeboYarpCamera::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    if (!m_yarp.checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
        std::cerr << "GazeboYarpCamera::Load error: yarp network does not seem to be available, is the yarpserver running?"<<std::endl;
        return;
    }
    
    std::cout << "*** GazeboYarpCamera plugin started ***" << std::endl;
    
    if (!_sensor) {
        gzerr << "GazeboYarpCamera plugin requires a CameraSensor." << std::endl;
        return;
    }

    _sensor->SetActive(true);
    
    // Add my gazebo device driver to the factory.
    ::yarp::dev::Drivers::factory().add(new ::yarp::dev::DriverCreatorOf< ::yarp::dev::GazeboYarpCameraDriver>
                                        ("gazebo_camera", "grabber", "GazeboYarpCameraDriver"));
    
    
    //Getting .ini configuration file from sdf
    bool configuration_loaded = false;
    
    if (_sdf->HasElement("yarpConfigurationFile")) {
        std::string ini_file_name = _sdf->Get<std::string>("yarpConfigurationFile");
        std::string ini_file_path = gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);
        
        if (ini_file_path != "" && m_parameters.fromConfigFile(ini_file_path.c_str())) {
            std::cout << "Found yarpConfigurationFile: loading from " << ini_file_path << std::endl;
            configuration_loaded = true;
        }
    }
    
    if (!configuration_loaded) {
        std::cout << "File .ini not found, quitting" << std::endl;
        return;
    }
    
    m_sensorName = _sensor->GetScopedName();
    m_sensor = (gazebo::sensors::CameraSensor*)_sensor.get();
    if(m_sensor == NULL)
    {
        std::cout << "m_sensor == NULL" << std::endl;
    }


    // Don't forget to load the camera plugin!!!! ???
//    CameraPlugin::Load(_sensor, _sdf);

    std::cout << "sensor scoped name is " << m_sensorName.c_str() << std::endl;
    //Insert the pointer in the singleton handler for retriving it in the yarp driver
    GazeboYarpPlugins::Handler::getHandler()->setSensor(_sensor.get());
    
    m_parameters.put(YarpScopedName.c_str(), m_sensorName.c_str());
    
    //Open the driver
    if (m_cameraDriver.open(m_parameters)) {
        std::cout << "Loaded GazeboYarpCamera Plugin correctly" << std::endl;
    } else {
        std::cout << "GazeboYarpCamera Plugin Load failed: error in opening yarp driver" << std::endl;
    }
    
    std::cout << "Trying to get the GazeboYarpCameraDriver interface from the device" << std::endl;

    m_cameraDriver.view(iFrameGrabberImage);
    if(iFrameGrabberImage == NULL)
    {
        std::cout << "Unable to get the iFrameGrabberImage interface from the device" << std::endl;
        return;
    }

    std::cout << "GazeboYarpCamera parameters" << std::endl;
    std::cout << m_parameters.toString() << std::endl;
}
    
}
