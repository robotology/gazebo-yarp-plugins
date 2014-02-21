/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alberto Cardellino and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "gazebo_yarp_plugins/Handler.hh"
#include <gazebo/physics/Entity.hh>
#include <gazebo/sensors/sensors.hh>

using namespace std;
using namespace gazebo;

GazeboYarpPluginHandler* GazeboYarpPluginHandler::_handle = NULL;
yarp::os::Semaphore GazeboYarpPluginHandler::_mutex = 1;


GazeboYarpPluginHandler::GazeboYarpPluginHandler()
{
    _robotMap.clear();
    _sensorsMap.clear();
    _handle = NULL;
}

GazeboYarpPluginHandler * GazeboYarpPluginHandler::getHandler()
{
    _mutex.wait();
    if (NULL == _handle)
    {
        cout << "Calling GazeboYarpPluginHandler Constructor";
        _handle = new GazeboYarpPluginHandler();
        if (NULL == _handle)
            cout << "Error while calling GazeboYarpPluginHandler constructor";

    }
    _mutex.post();

    return _handle;
}

bool GazeboYarpPluginHandler::setRobot(gazebo::physics::Model* _model)
{
    bool ret = false;
    std::string scopedRobotName = _model->GetScopedName();
    cout << "GazeboYarpPluginHandler: Inserting Robot : " << scopedRobotName << endl;
    
    RobotsMap::iterator robot = _robotMap.find(scopedRobotName);
    if (robot != _robotMap.end()) {
        //robot already exists. Increment reference counting
        robot->second.incrementCount();
        cout << "Robot already registered, pointers match." << endl;
        ret = true;
    }
    else {
        //robot does not exists. Add to map
        ReferenceCountingModel model(_model);
        if(!_robotMap.insert(std::pair<std::string, ReferenceCountingModel>(scopedRobotName, model)).second) {
            cout << "Error in GazeboYarpPluginHandler while inserting a new sensor pointer!\n";
            cout << " The name of the sensor is already present but the pointer does not match with the one already registered!!\n";
            cout << " This should not happen, as the scoped name should be unique in Gazebo. Fatal error." << endl;
            ret = false;
        }
        else {
            ret = true;
            cout << "Singleton: Added a new robot " << scopedRobotName << ".\n";
        }
    }

    return ret;
}

gazebo::physics::Model* GazeboYarpPluginHandler::getRobot(std::string robotName)
{
    gazebo::physics::Model* tmp = NULL;
    cout << "Looking for robot : " << robotName << endl;
    
    RobotsMap::iterator robot = _robotMap.find(robotName);
    if (robot != _robotMap.end()) {
        cout << "Robot " << robotName << " was happily found!\n";
        tmp = robot->second.object();
    }
    else {
        cout << "Robot was not found: " << robotName << endl;
        tmp = NULL;
    }
    return tmp;
}

void GazeboYarpPluginHandler::removeRobot(std::string robotName)
{
    RobotsMap::iterator robot = _robotMap.find(robotName);
    if (robot != _robotMap.end()) {
        robot->second.decrementCount();
        if (!robot->second.count()) {
            cout << "Removing robot " << robotName << std::endl;
            _robotMap.erase(robot);
        }
    }
    else {
        cout << "Could not remove robot " << robotName << ". Robot was not found" << std::endl;
    }
}

bool GazeboYarpPluginHandler::setSensor(gazebo::sensors::Sensor* _sensor)
{
    bool ret = false;
    std::string scopedSensorName = _sensor->GetScopedName();
    cout << "GazeboYarpPluginHandler: Inserting Sensor : " << scopedSensorName << endl;
    
    SensorsMap::iterator sensor = _sensorsMap.find(scopedSensorName);
    if (sensor != _sensorsMap.end()) {
        //sensor already exists. Increment reference counting
        sensor->second.incrementCount();
        cout << "Sensor already registered, pointers match." << endl;
        ret = true;
    }
    else {
        //sensor does not exists. Add to map
        ReferenceCountingSensor countedSensor(_sensor);
        if(!_sensorsMap.insert(std::pair<std::string, ReferenceCountingSensor>(scopedSensorName, countedSensor)).second) {
            cout << "Error in GazeboYarpPluginHandler while inserting a new sensor pointer!\n";
            cout << " The name of the sensor is already present but the pointer does not match with the one already registered!!\n";
            cout << " This should not happen, as the scoped name should be unique in Gazebo. Fatal error." << endl;
            ret = false;
        }
        else {
            ret = true;
            cout << "Singleton: Added a new sensor " << scopedSensorName << ".\n";
        }
    }
    
    return ret;
}
    
// return the sensor pointer given the sensor scoped namespac
gazebo::sensors::Sensor* GazeboYarpPluginHandler::getSensor(const std::string sensorScopedName)
{
    gazebo::sensors::Sensor* tmp = NULL;
    cout << "Looking for sensor : " << sensorScopedName << endl;
    
    SensorsMap::iterator sensor = _sensorsMap.find(sensorScopedName);
    if (sensor != _sensorsMap.end()) {
        cout << "Sensor " << sensorScopedName << " was happily found!\n";
        tmp = sensor->second.object();
    }
    else {
        cout << "Sensor was not found: " << sensorScopedName << endl;
        tmp = NULL;
    }
    return tmp;
}

void GazeboYarpPluginHandler::removeSensor(const std::string sensorName)
{
    SensorsMap::iterator sensor = _sensorsMap.find(sensorName);
    if (sensor != _sensorsMap.end()) {
        sensor->second.decrementCount();
        if (!sensor->second.count()) {
            cout << "Removing sensor " << sensorName << std::endl;
            _sensorsMap.erase(sensor);
        }
    }
    else {
        cout << "Could not remove sensor " << sensorName << ". Sensor was not found" << std::endl;
    }
}
