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

#include <mutex>

namespace gazebo
{
    namespace sensors {
        class Sensor;
    }
    namespace physics {
        class Model;
    }
}

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>

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
     * The YARP devices are stored in this database using the following schema:
     *  * For YARP devices created by Model plugins, the deviceDatabaseKey is 
     *    defined as deviceDatabaseKey = Model::GetScopedName() + "::" + yarpDeviceName
     *  * For YARP devices created by Sensor plugins, the deviceDatabaseKey is 
     *    defined as deviceDatabaseKey = Sensor::GetScopedName() + "::" + yarpDeviceName
     *
     * yarpDeviceName is a non-scoped identifier of the specific instance of the YARP device, 
     * that is tipically specified by the Gazebo plugin configuration file, and corresponds to the 
     * name attribute of the device XML element when the device is created with the robotinterface 
     * XML format.
     *
     * If the device with the same deviceDatabaseKey exists and the pointer are the same return success, 
     * if pointers doesn't match returns error.
     * \param deviceDatabaseKey the deviceDatabaseKey of the device to be added to the internal database
     * \param device2add the pointer of the device to be added to the internal database
     * \return true if successfully added, or the device already exists. False otherwise.
     */
    bool setDevice(std::string deviceDatabaseKey, yarp::dev::PolyDriver* device2add);

    /** Returns the pointer to the device matching the deviceDatabaseKey
     * \param deviceDatabaseKey deviceDatabaseKey to be looked for
     * \return the pointer to the device
     */
    yarp::dev::PolyDriver* getDevice(const std::string& deviceDatabaseKey) const;

    /** \brief Removes a device from the internal database
     *  \param deviceDatabaseKey the deviceDatabaseKey of the device to be removed
     */
    void removeDevice(const std::string& deviceDatabaseKey);

    /** 
     * \brief Returns a list of the opened devices
     * \note This list acts just as a view of the available devices, 
     *       and it does not transfer or share ownership of the devices.
     *       The consumer code needs to make sure that the driver lifetime
     *       is longer then the consumer lifetime.
     *
     * This method returns all the YARP devices that have been created by the specified model, 
     * and by all its nested model and sensors. As the PolyDriverList is effectively a map in which 
     * the key is a string and the value is the PolyDriver pointer, in this case the key of the PolyDriverList
     * is the yarpDeviceName without any scope, i.e. not the deviceDatabaseKey .
     * 
     * If after removing the scope two devices have the same yarpDeviceName, the getModelDevicesAsPolyDriverList
     * prints an error and returns false, while true is returned if everything works as expected.
     */
    bool getDevicesAsPolyDriverList(const std::string& modelScopedName, yarp::dev::PolyDriverList& list, std::vector<std::string>& deviceScopedNames);
    
    /** 
     * \brief Decrease the usage count for the devices that are acquired with the getDevicesAsPolyDriverList
     * 
     * As Gazebo plugins are not destructed in the same order that they are loaded, it is necessary to keep 
     * a count of the users of each device, to ensure that is only destructed when no device are still attached to it.
     * 
     * This function needs to be called by any plugin that has called the getDevicesAsPolyDriverList method during
     * the unload/destruction process.
     */
    void releaseDevicesInList(const std::vector<std::string>& deviceScopedNames);

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
    static std::mutex& mutex();


    Handler();
    RobotsMap m_robotMap;      // map of known robots
    SensorsMap m_sensorsMap;    // map of known sensors
    DevicesMap m_devicesMap;    // map of known yarp devices

    bool findRobotName(sdf::ElementPtr sdf, std::string* robotName);

};

}

#endif  // GAZEBOYARP_HANDLER_HH
