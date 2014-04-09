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

namespace GazeboYarpPlugins {

Handler* Handler::_handle = NULL;
yarp::os::Semaphore Handler::_mutex = 1;


Handler::Handler()
{
    _robotMap.clear();
    _sensorsMap.clear();
    _handle = NULL;
}

Handler * Handler::getHandler()
{
    _mutex.wait();
    if (NULL == _handle)
    {
        cout << "Calling GazeboYarpPlugins::Handler Constructor";
        _handle = new Handler();
        if (NULL == _handle)
            cout << "Error while calling GazeboYarpPluginHandler constructor";

    }
    _mutex.post();

    return _handle;
}

bool Handler::setRobot(gazebo::physics::Model* _model)
{
    bool ret = false;
    std::string scopedRobotName = _model->GetScopedName();
    cout << "GazeboYarpPlugins::Handler: Inserting Robot : " << scopedRobotName << endl;
    
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
            cout << "Error in GazeboYarpPlugins::Handler while inserting a new sensor pointer!\n";
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

gazebo::physics::Model* Handler::getRobot(std::string robotName)
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

void Handler::removeRobot(std::string robotName)
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

bool Handler::setSensor(gazebo::sensors::Sensor* _sensor)
{
    bool ret = false;
    std::string scopedSensorName = _sensor->GetScopedName();
    cout << "GazeboYarpPlugins::Handler: Inserting Sensor : " << scopedSensorName << endl;
    
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
            cout << "Error in GazeboYarpPlugins::Handler while inserting a new sensor pointer!\n";
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
gazebo::sensors::Sensor* Handler::getSensor(const std::string sensorScopedName)
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

void Handler::removeSensor(const std::string sensorName)
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
}
