/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alberto Cardellino and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "Handler.hh"

#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <gazebo/physics/Entity.hh>
#include <gazebo/sensors/sensors.hh>

#include <unordered_map> 

using namespace gazebo;

namespace GazeboYarpPlugins {

Handler* Handler::s_handle = NULL;

std::mutex& Handler::mutex()
{
    static std::mutex s_mutex;
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
    std::lock_guard<std::mutex> lock(mutex());
    if (!s_handle) {
        s_handle = new Handler();
        if (!s_handle)
            yError() << "Error while calling GazeboYarpPluginHandler constructor";
    }

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
            yError() << "Error in GazeboYarpPlugins::Handler while inserting a new sensor pointer!";
            yError() << " The name of the sensor is already present but the pointer does not match with the one already registered!";
            yError() << " This should not happen, as the scoped name should be unique in Gazebo. Fatal error.";
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
        yError() << "Robot was not found: " << robotName;
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
        yError() << "Could not remove robot " << robotName << ". Robot was not found";
    }
}

bool Handler::setSensor(gazebo::sensors::Sensor* _sensor)
{
    bool ret = false;
#if GAZEBO_MAJOR_VERSION >= 7
    std::string scopedSensorName = _sensor->ScopedName();
#else
    std::string scopedSensorName = _sensor->GetScopedName();
#endif
    SensorsMap::iterator sensor = m_sensorsMap.find(scopedSensorName);
    if (sensor != m_sensorsMap.end()) {
        //sensor already exists. Increment reference counting
        sensor->second.incrementCount();
        ret = true;
    } else {
        //sensor does not exists. Add to map
        ReferenceCountingSensor countedSensor(_sensor);
        if (!m_sensorsMap.insert(std::pair<std::string, ReferenceCountingSensor>(scopedSensorName, countedSensor)).second) {
            yError() << "Error in GazeboYarpPlugins::Handler while inserting a new sensor pointer!";
            yError() << " The name of the sensor is already present but the pointer does not match with the one already registered!";
            yError() << " This should not happen, as the scoped name should be unique in Gazebo. Fatal error.";
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
        yError() << "Sensor was not found: " << sensorScopedName;
        tmp = NULL;
    }
    return tmp;
}

std::vector<std::string> Handler::getSensors() const
{
    // define the output vector of sensor names
    std::vector<std::string> sensorsV(m_sensorsMap.size());
    
    // iterate over the sensor map and get all the names
    SensorsMap::const_iterator sensor; int idx;
    for (sensor=m_sensorsMap.begin(), idx=0;
         sensor!=m_sensorsMap.end();
         sensor++,idx++)
    {
        sensorsV[idx] = sensor->first;
    }
    
    return sensorsV;
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
        yError() << "Could not remove sensor " << sensorName << ". Sensor was not found";
    }
}

bool Handler::setDevice(std::string deviceDatabaseKey, yarp::dev::PolyDriver* device2add)
{
    bool ret = false;
    DevicesMap::iterator device = m_devicesMap.find(deviceDatabaseKey);
    if (device != m_devicesMap.end()) {
        //device already exists. Increment reference counting
        if(device->second.object() == device2add)
        {
            device->second.incrementCount();
            ret = true;
        }
        else
        {
            yError() << " Error in GazeboYarpPlugins::Handler while inserting a new yarp device pointer!";
            yError() << " The name of the device is already present but the pointer does not match with the one already registered!";
            yError() << " This should not happen, check the names are correct in your config file. Fatal error.";
        }
    } else {
        //device does not exists. Add to map
        ReferenceCountingDevice countedDevice(device2add);
        if (!m_devicesMap.insert(std::pair<std::string, ReferenceCountingDevice>(deviceDatabaseKey, countedDevice)).second) {
            yError() << " Error in GazeboYarpPlugins::Handler while inserting a new device pointer!";
            ret = false;
        } else {
            ret = true;
        }
    }
    return ret;
}

yarp::dev::PolyDriver* Handler::getDevice(const std::string& deviceDatabaseKey) const
{
    yarp::dev::PolyDriver* tmp = NULL;

    DevicesMap::const_iterator device = m_devicesMap.find(deviceDatabaseKey);
    if (device != m_devicesMap.end()) {
        tmp = device->second.object();
    } else {
        tmp = NULL;
    }
    return tmp;
}

void Handler::removeDevice(const std::string& deviceDatabaseKey)
{
    DevicesMap::iterator device = m_devicesMap.find(deviceDatabaseKey);
    if (device != m_devicesMap.end()) {
        device->second.decrementCount();
        if (!device->second.count()) {
            device->second.object()->close();
            m_devicesMap.erase(device);
        }
    } else {
        yError() << "Could not remove device " << deviceDatabaseKey << ". Device was not found";
    }
    return;
}

inline bool startsWith(const std::string&completeString, 
                       const std::string&candidatePrefix)
{
    // https://stackoverflow.com/a/40441240
    return (completeString.rfind(candidatePrefix, 0) == 0);
} 

bool Handler::getDevicesAsPolyDriverList(const std::string& modelScopedName, yarp::dev::PolyDriverList& list, std::vector<std::string>& deviceScopedNames)
{
    deviceScopedNames.resize(0);

    list = yarp::dev::PolyDriverList();

    // This map contains only the yarpDeviceName that we actually added
    // to the returned yarp::dev::PolyDriverList
    std::unordered_map<std::string, std::string> inserted_yarpDeviceName2deviceDatabaseKey;

    for (auto&& devicesMapElem: m_devicesMap) {
        std::string deviceDatabaseKey = devicesMapElem.first;

        std::string yarpDeviceName;
        
        // If the deviceDatabaseKey starts with the modelScopedName (device spawned by model plugins), 
        // or by  "default::" + modelScopedName (device spawned by sensor plugins) then it is eligible for insertion
        // in the returned list
        if (startsWith(deviceDatabaseKey, modelScopedName) || startsWith(deviceDatabaseKey, "default::" + modelScopedName)) {
            // Extract yarpDeviceName from deviceDatabaseKey
            yarpDeviceName = deviceDatabaseKey.substr(deviceDatabaseKey.find_last_of(":")+1);

            // Check if a device with the same yarpDeviceName was already inserted
            auto got = inserted_yarpDeviceName2deviceDatabaseKey.find(yarpDeviceName);

            // If not found, insert and continue
            if (got == inserted_yarpDeviceName2deviceDatabaseKey.end()) {
                // If no name collision is found, insert and continue
                inserted_yarpDeviceName2deviceDatabaseKey.insert({yarpDeviceName, deviceDatabaseKey});
                list.push(devicesMapElem.second.object(), yarpDeviceName.c_str());
                deviceScopedNames.push_back(deviceDatabaseKey);
                // Increase usage counter
                setDevice(deviceDatabaseKey, devicesMapElem.second.object());
            } else {
                // If a name collision is found, print a clear error and return
                yError() << "GazeboYARPPlugins robotinterface getDevicesAsPolyDriverList error: ";
                yError() << "two YARP devices with yarpDeviceName " << yarpDeviceName 
                         << " found in model " << modelScopedName;
                yError() << "First instance: " << got->second;
                yError() << "Second instance: " << deviceDatabaseKey;
                yError() << "Please eliminate or rename one of the two instances. ";
                list = yarp::dev::PolyDriverList();
                releaseDevicesInList(deviceScopedNames);
                deviceScopedNames.resize(0);
                return false;
            }

        }

    }

    return true;
}


void Handler::releaseDevicesInList(const std::vector<std::string>& deviceScopedNames)
{
    for (auto&& deviceScopedName: deviceScopedNames) {
        removeDevice(deviceScopedName);
    }
    return;
}


}
