/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef __GAZEBO_YARP_PLUGIN_HANDLER_HH__
#define __GAZEBO_YARP_PLUGIN_HANDLER_HH__

#include <map>
#include <yarp/os/Semaphore.h>
#include <gazebo/physics/Entity.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo
{
    class GazeboYarpPluginHandler;
}

typedef std::map<std::string, gazebo::physics::Model*> RobotsMap;
typedef std::map<std::string, gazebo::sensors::Sensor*> SensorsMap;

class gazebo::GazeboYarpPluginHandler
{
public:
    // static method for using the handler
    static GazeboYarpPluginHandler* getHandler();

    // add a new modelPointer to the "database", if it already exists and the pointer are the same return success,
    // if pointers doesn't match returns error.
    bool setRobot(gazebo::physics::Model* _model);

    // return the model pointer given the robot name
    gazebo::physics::Model* getRobot(std::string robotName);

    // add a new sensorPointer to the "database", if the sensor already exists and the pointer are the same return success,
    // if pointers doesn't match returns error.
    // the key used in the "database" is the scoped name of the sensor
    bool setSensor(gazebo::sensors::Sensor* _sensor);
    
    // return the sensor pointer given the sensor scoped namespac
    gazebo::sensors::Sensor* getSensor(const std::string sensorScopedName);

    ~GazeboYarpPluginHandler();

private:
    // singleton stuff
    static yarp::os::Semaphore          _mutex;
    static GazeboYarpPluginHandler*     _handle;

    GazeboYarpPluginHandler();
    RobotsMap                           _robotMap;      // map of known robots
    SensorsMap                          _sensorsMap;    // map of known sensors

    bool findRobotName(sdf::ElementPtr sdf, std::string *robotName);

};


#endif  // __GAZEBO_YARP_PLUGIN_HANDLER_HH__
