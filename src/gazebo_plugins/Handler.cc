/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alberto Cardellino and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "gazebo_yarp_plugins/Handler.hh"

using namespace std;
using namespace gazebo;

GazeboYarpPluginHandler* GazeboYarpPluginHandler::_handle = NULL;
yarp::os::Semaphore GazeboYarpPluginHandler::_mutex = 1;


GazeboYarpPluginHandler::GazeboYarpPluginHandler()
{
    _robotMap.clear();
    _handle = NULL;
}

GazeboYarpPluginHandler * GazeboYarpPluginHandler::getHandler()
{
    _mutex.wait();
    if (NULL == _handle)
    {
        cout << "Calling EthManager Constructor";
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
    std::string scoped_robot_name = _model->GetScopedName();
    cout << "GazeboYarpPluginHandler: Inserting Robot : " << scoped_robot_name << endl;

    if( ! _robotMap.insert(std::pair<std::string, gazebo::physics::Model*>(scoped_robot_name, _model) ).second)
    {
        // the element could be already present, check it out just to be sure!
        if(_model != _robotMap.at(scoped_robot_name) )
        {
            cout << "Error in GazeboYarpPluginHandler while inserting a new sensor pointer!\n";
            cout << " The name of the sensor is already present but the pointer does not match with the one already registered!!\n";
            cout << " This should not happen, as the scoped name should be unique in Gazebo. Fatal error." << endl;
            ret = false;
        }
        else
        {
            cout << "Robot already registered, pointers match." << endl;
            ret = true;
        }
    }
    else
    {
        ret = true;
        cout << "Singleton: Added a new robot " << scoped_robot_name << ".\n";
    }
    return ret;
}

gazebo::physics::Model* GazeboYarpPluginHandler::getRobot(std::string robotName)
{
    gazebo::physics::Model* tmp = NULL;
    cout << "Looking for robot : " << robotName << endl;
    try
    {
        tmp = _robotMap.at(robotName);
    }
    catch (...)
    {
         cout << "Robot was not found: " << robotName << endl;
         tmp = NULL;
    }
    if(NULL != tmp)
        cout << "Robot " << robotName << " was happily found!\n";
    return tmp;
}


bool GazeboYarpPluginHandler::setSensor(gazebo::sensors::Sensor* _sensor)
{
    bool ret = false;
    std::string scoped_sensor_name = _sensor->GetScopedName();
    cout << "GazeboYarpPluginHandler: Inserting Sensor : " << scoped_sensor_name << endl;

    if( ! _sensorsMap.insert(std::pair<std::string, gazebo::sensors::Sensor*>(scoped_sensor_name, _sensor) ).second)
    {
        // the element could be already present, check it out just to be sure!
        if(_sensor != _sensorsMap.at(scoped_sensor_name) )
        {
            cout << "Error in GazeboYarpPluginHandler while inserting a new sensor pointer!\n";
            cout << " The name of the sensor is already present but the pointer does not match with the one already registered!!\n";
            cout << " This should not happen, as the scoped name should be unique in Gazebo. Fatal error." << endl;
            ret = false;
        }
        else
        {
            cout << "Sensor already registered, pointers match. This however should not happen. This is a debug msg and can be removed." << endl;
            ret = true;
        }
    }
    else
    {
        ret = true;
        cout << "Singleton: Added a new sensor " << scoped_sensor_name << ".\n";
    }
    return ret;
}
    
// return the sensor pointer given the sensor scoped namespac
gazebo::sensors::Sensor* GazeboYarpPluginHandler::getSensor(const std::string sensorScopedName)
{
    gazebo::sensors::Sensor* tmp = NULL;
    cout << "Looking for sensor : " << sensorScopedName << endl;
    try
    {
        tmp = _sensorsMap.at(sensorScopedName);
    }
    catch (...)
    {
         cout << "Sensor was not found: " << sensorScopedName << endl;
         tmp = NULL;
    }
    if(NULL != tmp)
        cout << "Sensor " << sensorScopedName << " was happily found!\n";
    return tmp;
}