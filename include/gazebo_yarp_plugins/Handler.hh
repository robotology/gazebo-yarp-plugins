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

//    namespace sensors {
//        class Sensor;
//    }
//    namespace physics {
//        class Model;
//    }
}

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
    
    /** \brief Removes a robot from the internal database
     *  \param robotName the name of the robot to be removed
     */
    void removeRobot(std::string robotName);

    // add a new sensorPointer to the "database", if the sensor already exists and the pointer are the same return success,
    // if pointers doesn't match returns error.
    // the key used in the "database" is the scoped name of the sensor
    bool setSensor(gazebo::sensors::Sensor* _sensor);
    
    // return the sensor pointer given the sensor scoped namespac
    gazebo::sensors::Sensor* getSensor(const std::string sensorScopedName);
    
    /** \brief Removes a sensor from the internal database
     *  \param sensorScopedName the name of the sensor to be removed
     */
    void removeSensor(const std::string sensorName);

    ~GazeboYarpPluginHandler();

private:
    
    template <class T>
    class ReferenceCountingObject
    {
        T& _object;
        unsigned short _count;
    public:
        ReferenceCountingObject(T& object):_object(object), _count(1) {}
        
        T& object() { return _object; }
        unsigned short count() { return _count; }
        void incrementCount() { _count++; }
        void decrementCount() { _count--; }
    };
    
    typedef ReferenceCountingObject<gazebo::physics::Model*> ReferenceCountingModel;
    typedef ReferenceCountingObject<gazebo::sensors::Sensor*> ReferenceCountingSensor;
    
    typedef std::map<std::string, ReferenceCountingModel> RobotsMap;
    typedef std::map<std::string, ReferenceCountingSensor> SensorsMap;
    
    // singleton stuff
    static yarp::os::Semaphore          _mutex;
    static GazeboYarpPluginHandler*     _handle;

    GazeboYarpPluginHandler();
    RobotsMap                           _robotMap;      // map of known robots
    SensorsMap                          _sensorsMap;    // map of known sensors

    bool findRobotName(sdf::ElementPtr sdf, std::string *robotName);
    
};


#endif  // __GAZEBO_YARP_PLUGIN_HANDLER_HH__
