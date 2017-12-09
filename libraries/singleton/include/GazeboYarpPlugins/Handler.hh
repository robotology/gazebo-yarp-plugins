/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_HANDLER_HH
#define GAZEBOYARP_HANDLER_HH

#include <map>

#include <sdf/sdf_config.h>

#if SDF_MAJOR_VERSION >= 3

#include <sdf/Element.hh>

#else

#include <boost/shared_ptr.hpp>
namespace sdf {
   class Element;
   typedef boost::shared_ptr<Element> ElementPtr;
}

#endif

#include <yarp/os/Semaphore.h>

namespace gazebo
{
    namespace sensors {
        class Sensor;
    }
    namespace physics {
        class Model;
    }
}

namespace yarp {
    namespace dev {
        class PolyDriver;
    }
}

namespace GazeboYarpPlugins {

/** \class Handler
 *
 * Singleton object class. You can use this class to save robots and sensors and retrieve it in other part of the application
 */
class Handler
{
public:
    /** Returns the singleton intance of Handler class and creates it if it does not exist.
     * \return the singleton handler
     */
    static Handler* getHandler();

    /** \brief Adds a new modelPointer to the "database".
     *
     * If it already exists and the pointer are the same return success,
     * if pointers doesn't match returns error.
     * \param _model the model to be added to the internal database
     * \return true if successfully added, or the model already exists. False otherwise.
     */
    bool setRobot(gazebo::physics::Model* _model);

    /** Returns the pointer to the model matching the robot name
     * \param robotName robot name to be looked for
     * \return the model matching the passed name
     */
    gazebo::physics::Model* getRobot(const std::string& robotName) const;

    /** \brief Removes a robot from the internal database
     *  \param robotName the name of the robot to be removed
     */
    void removeRobot(const std::string& robotName);

    /** \brief Adds a new sensorPointer to the "database".
     *
     *  If the sensor already exists and the pointer are the same return success, if pointers doesn't match returns error.
     * \param _sensor the sensor to be added to the internal database
     * \return true if successfully added, or the model already exists. False otherwise.
     */
    bool setSensor(gazebo::sensors::Sensor* _sensor);

    /** Returns the  pointer to the sensor matching the sensor name
     * \param sensorScopedName sensor name to be looked for
     * \return the sensor matching the passed name
     */
    gazebo::sensors::Sensor* getSensor(const std::string& sensorScopedName) const;

    /** Returns the vector of all sensor names
     * \return vector of all sensor names
     */
    std::vector<std::string> getSensors() const;
    
    /** \brief Removes a sensor from the internal database
     *  \param sensorName the name of the sensor to be removed
     */
    void removeSensor(const std::string& sensorName);

    /** \brief Adds a new yarp device pointer to the "database".
     *
     *  If the device already exists and the pointer are the same return success, if pointers doesn't match returns error.
     * \param deviceName the name of the device to be added to the internal database
     * \param device2add the pointer of the device to be added to the internal database
     * \return true if successfully added, or the device already exists. False otherwise.
     */
    bool setDevice(std::string deviceName, yarp::dev::PolyDriver* device2add);

    /** Returns the pointer to the device matching the sensor name
     * \param deviceName device name to be looked for
     * \return the pointer to the device
     */
    yarp::dev::PolyDriver* getDevice(const std::string& deviceName) const;

    /** \brief Removes a device from the internal database
     *  \param deviceName the name of the device to be removed
     */
    void removeDevice(const std::string& deviceName);

    /** Destructor
     */
    ~Handler();

private:

    template <class T>
    class ReferenceCountingObject
    {
        T m_object;
        unsigned short m_count;
    public:
        ReferenceCountingObject(T object): m_object(object), m_count(1) {}

        T object() const { return m_object; }
        unsigned short count() const { return m_count; }
        void incrementCount() { m_count++; }
        void decrementCount() { m_count--; }
    };

    typedef ReferenceCountingObject<gazebo::physics::Model*> ReferenceCountingModel;
    typedef ReferenceCountingObject<gazebo::sensors::Sensor*> ReferenceCountingSensor;
    typedef ReferenceCountingObject<yarp::dev::PolyDriver*> ReferenceCountingDevice;

    typedef std::map<std::string, ReferenceCountingModel> RobotsMap;
    typedef std::map<std::string, ReferenceCountingSensor> SensorsMap;
    // store list of yarp decices
    typedef std::map<std::string, ReferenceCountingDevice> DevicesMap;

    // singleton stuff
    static Handler* s_handle;
    static yarp::os::Semaphore& mutex();


    Handler();
    RobotsMap m_robotMap;      // map of known robots
    SensorsMap m_sensorsMap;    // map of known sensors
    DevicesMap m_devicesMap;    // map of known yarp devices

    bool findRobotName(sdf::ElementPtr sdf, std::string* robotName);

};

}

#endif  // GAZEBOYARP_HANDLER_HH
