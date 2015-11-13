/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alberto Cardellino and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "Handler.hh"

#include <yarp/dev/PolyDriver.h>
#include <gazebo/physics/Entity.hh>
#include <gazebo/sensors/sensors.hh>

using namespace gazebo;

namespace GazeboYarpPlugins {

Handler* Handler::s_handle = NULL;

yarp::os::Semaphore& Handler::mutex()
{
    static yarp::os::Semaphore s_mutex(1);
    return s_mutex;
}

Handler::Handler() : m_robotMap(), m_sensorsMap(), m_devicesMap()
{
    m_robotMap.clear();
    m_sensorsMap.clear();
    m_devicesMap.clear();
}

Handler* Handler::getHandler()
{
    mutex().wait();
    if (!s_handle) {
        s_handle = new Handler();
        if (!s_handle)
            std::cout << "Error while calling GazeboYarpPluginHandler constructor" << std::endl;
    }
    mutex().post();

    return s_handle;
}

bool Handler::setRobot(gazebo::physics::Model* _model)
{
    bool ret = false;
    std::string scopedRobotName = _model->GetScopedName();

    RobotsMap::iterator robot = m_robotMap.find(scopedRobotName);
    if (robot != m_robotMap.end()) {
        //robot already exists. Increment reference counting
        robot->second.incrementCount();
        ret = true;
    }
    else {
        //robot does not exists. Add to map
        ReferenceCountingModel model(_model);
        if (!m_robotMap.insert(std::pair<std::string, ReferenceCountingModel>(scopedRobotName, model)).second) {
            std::cout << "Error in GazeboYarpPlugins::Handler while inserting a new sensor pointer!" << std::endl;
            std::cout << " The name of the sensor is already present but the pointer does not match with the one already registered!" << std::endl;
            std::cout << " This should not happen, as the scoped name should be unique in Gazebo. Fatal error." << std::endl;
            ret = false;
        } else {
            ret = true;
        }
    }
    return ret;
}

gazebo::physics::Model* Handler::getRobot(const std::string& robotName) const
{
    gazebo::physics::Model* tmp = NULL;

    RobotsMap::const_iterator robot = m_robotMap.find(robotName);
    if (robot != m_robotMap.end()) {
        tmp = robot->second.object();
    }
    else {
        std::cout << "Robot was not found: " << robotName << std::endl;
        tmp = NULL;
    }
    return tmp;
}

void Handler::removeRobot(const std::string& robotName)
{
    RobotsMap::iterator robot = m_robotMap.find(robotName);
    if (robot != m_robotMap.end()) {
        robot->second.decrementCount();
        if (!robot->second.count()) {
            m_robotMap.erase(robot);
        }
    } else {
        std::cout << "Could not remove robot " << robotName << ". Robot was not found" << std::endl;
    }
}

bool Handler::setSensor(gazebo::sensors::Sensor* _sensor)
{
    bool ret = false;
    std::string scopedSensorName = _sensor->GetScopedName();

    SensorsMap::iterator sensor = m_sensorsMap.find(scopedSensorName);
    if (sensor != m_sensorsMap.end()) {
        //sensor already exists. Increment reference counting
        sensor->second.incrementCount();
        ret = true;
    } else {
        //sensor does not exists. Add to map
        ReferenceCountingSensor countedSensor(_sensor);
        if (!m_sensorsMap.insert(std::pair<std::string, ReferenceCountingSensor>(scopedSensorName, countedSensor)).second) {
            std::cout << "Error in GazeboYarpPlugins::Handler while inserting a new sensor pointer!" << std::endl;
            std::cout << " The name of the sensor is already present but the pointer does not match with the one already registered!" << std::endl;
            std::cout << " This should not happen, as the scoped name should be unique in Gazebo. Fatal error." << std::endl;
            ret = false;
        } else {
            ret = true;
        }
    }
    return ret;
}

// return the sensor pointer given the sensor scoped namespac
gazebo::sensors::Sensor* Handler::getSensor(const std::string& sensorScopedName) const
{
    gazebo::sensors::Sensor* tmp = NULL;

    SensorsMap::const_iterator sensor = m_sensorsMap.find(sensorScopedName);
    if (sensor != m_sensorsMap.end()) {
        tmp = sensor->second.object();
    } else {
        std::cout << "Sensor was not found: " << sensorScopedName << std::endl;
        tmp = NULL;
    }
    return tmp;
}

void Handler::removeSensor(const std::string& sensorName)
{
    SensorsMap::iterator sensor = m_sensorsMap.find(sensorName);
    if (sensor != m_sensorsMap.end()) {
        sensor->second.decrementCount();
        if (!sensor->second.count()) {
            m_sensorsMap.erase(sensor);
        }
    } else {
        std::cout << "Could not remove sensor " << sensorName << ". Sensor was not found" << std::endl;
    }
}

bool Handler::setDevice(std::string deviceName, yarp::dev::PolyDriver* device2add)
{
    bool ret = false;
    DevicesMap::iterator device = m_devicesMap.find(deviceName);
    if (device != m_devicesMap.end()) {
        //device already exists. Increment reference counting
        if(device->second.object() == device2add)
        {
            device->second.incrementCount();
            ret = true;
        }
        else
        {
            std::cout << " Error in GazeboYarpPlugins::Handler while inserting a new yarp device pointer!" << std::endl;
            std::cout << " The name of the device is already present but the pointer does not match with the one already registered!" << std::endl;
            std::cout << " This should not happen, check the names are correct in your config file. Fatal error." << std::endl;
        }
    } else {
        //device does not exists. Add to map
        ReferenceCountingDevice countedDevice(device2add);
        if (!m_devicesMap.insert(std::pair<std::string, ReferenceCountingDevice>(deviceName, countedDevice)).second) {
            std::cout << " Error in GazeboYarpPlugins::Handler while inserting a new device pointer!" << std::endl;
            ret = false;
        } else {
            ret = true;
        }
    }
    return ret;
}

yarp::dev::PolyDriver* Handler::getDevice(const std::string& deviceName) const
{
    yarp::dev::PolyDriver* tmp = NULL;

    DevicesMap::const_iterator device = m_devicesMap.find(deviceName);
    if (device != m_devicesMap.end()) {
        tmp = device->second.object();
    } else {
        tmp = NULL;
    }
    return tmp;
}

void Handler::removeDevice(const std::string& deviceName)
{
    DevicesMap::iterator device = m_devicesMap.find(deviceName);
    if (device != m_devicesMap.end()) {
        device->second.decrementCount();
        if (!device->second.count()) {
            device->second.object()->close();
            m_devicesMap.erase(device);
        }
        else
        {
            std::cout << "Not removing device '" << deviceName << "' yet because it is still used by other devices, Decremented counter to " << device->second.count() << std::endl;
        }
    } else {
        std::cout << "Could not remove device " << deviceName << ". Device was not found" << std::endl;
    }
    return;
}

}
