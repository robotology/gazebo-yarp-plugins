/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_CONTACTLOADCELLARRAYDRIVER_H
#define GAZEBOYARP_CONTACTLOADCELLARRAYDRIVER_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Semaphore.h>

#include <yarp/sig/Matrix.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>


#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/sensors/SensorManager.hh>

#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Inertial.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Matrix3.hh>
#include <ignition/math.hh>

namespace yarp
{
    namespace dev
    {
        class GazeboYarpContactLoadCellArrayDriver;
    }
}

/**
 * YARP Device Driver for an array of contact load cells giving 1-axis force at different locations
 * 
 * This device is used to simulate the contact normal forces that might be 
 * measured by an arbitrary number of load cells at specific locations with respect
 * to a given link's origin frame. It uses the instance of a Gazebo Contact Sensor which
 * provides contact information of the link (specifically providing wrenches - 
 * 6 axis force torque components,  acting at different position with respect to the link).
 * The equivalent wrench acts on the Center of Gravity(CG) of the link expressed in link origin frame.
 * The CG of thee link is defined by the <inertial > in the sdf as a child of <link> tag.
 * This information is transformed and mapped to the desired contact normal forces.
 * 
 * MAJOR ASSUMPTION: The load cells are distributed over a plane perpendicular to the z-axis of the link frame
 * and the load cells are measuring the force over the z-direction
 * 
 * In its current version, this device reads the location of the load cells (to be copied manually from the urdf or sdf onto the configuration file)
 * from a configuration file. Reads the contact forces and torques acting at the gazebo collision level through a 
 * gazebo ContactSensor instance. Finds an equivalent wrench acting at the CG of the specified link(location CG, rotation same as link origin). Transforms this
 * equivalent wrench to the origin of the specified link. This transformation is necessary since the load cell locations are
 * with respect to the link origin. Finally, the transformed equivalent wrench is mapped to an array of contact normal forces acting
 * at the load cell locations. 
 * 
 * This mapping problem is an infinite solution problem and is solved by obtaining a pseudo inverse, 
 * which gives the results with a minimized norm.
 * 
 * The configuation file must be specified in the sdf within the plugin element within the yarpConfigurationFile tags
 * The configuration file must contain two groups [WRAPPER] and [DRIVER]
 * 
 * Parameters accepted in the config argument of the open method for [WRAPPER]:
 * 
 * | Parameter name| Type |Units|Default Value|Required |                 Description                |              Notes             |
 * |:-------------:|:----:|:---:|:-----------:|:-------:|:------------------------------------------:|:------------------------------:|
 * | name          |string|  -  |    -        |   Yes   | full name of the port opened by the device | MUST start with a '/' character|
 * | period        |double|  ms |    20       |   No    | refresh period of broadcast value in ms    |    optional, default 20ms      |   
 * | device        |string|  -  |    -        |   Yes   | name of the network wrapper to connect to  | MUST be set to analogServer    |    
 *
 * Parameters accepted in the config argument of the open method for [DRIVER]:
 * 
 * | Parameter name|      Type     |Units|Default Value|Required |                       Description                      |                        Notes                     |
 * |:-------------:|:-------------:|:---:|:-----------:|:-------:|:------------------------------------------------------:|:------------------------------------------------:|
 * | device        |    string     |  -  |    -        |   Yes   | name of the device driver to instantiate               | MUST be set to gazebo_contactloadcellarray|
 * | loadCellNames |list of strings|  -  |    -        |   Yes   | list of load cell names, eg. (lr rr lf rf)             | locations MUST be obtained manually from urdf/sdf|
 * | loadCellX     |list of doubles|  m  |    -        |   Yes   | list of load cell location X coordinate wrt link origin| MUST be the same size and order as loadCellName  |
 * | loadCellY     |list of doubles|  m  |    -        |   Yes   | list of load cell location Y coordinate wrt link origin| MUST be the same size and order as loadCellName  |
 * | loadCellZ     |list of doubles|  m  |    -        |   Yes   | list of load cell location Z coordinate wrt link origin| MUST be the same size and order as loadCellName  |
 * | linkName      |     string    |  -  |    -        |   Yes   | name of the link associated to the contact sensor      |                            -                     |
 *
 * NOTE: Although being sensor-related, this plugin is implemented as a model plugin instead of a sensor plugin. This design choice is due to the fact that transformations related
 * to the link were required for computations in this device; which could not be achieved through a sensor plugin, since Gazebo inherently divides its
 * physics engine and sensor engine in its software architecture. As a model plugin, the link information can be directly accessed,
 * however the sensor name and collision name related to the link are obtained by parsing through the SDF directly. Again, this design decision
 * of parsing through the sdf is made due to the fact that the Link information does not give the type of sensors but only sensor counts.
 * 
 * This plugin in its current version assumes a only single collision element is associated to the specified link
 * This sensor plugin in its current version handles only one link with a plugin instance 
 * (i.e., multiple links cannot be specified per plugin, but multiple plugins can be instantiated through the sdf)
 * The collision element in the contact sensor added to the specific link in the sdf should be set to {link_name}_collision
 * The name of the collision element associated to the link should be set to default (i.e. "name" parameter should not be set so that it can take
 * the default name {link_name}_collision. Otherwise, things become worse).
 * 
 * WARNING: Please note that the current version of this plugin is not tested on (heavily) nested models and also rather does not support the models
 * having links with non-default collision element names.
 * Refer discussion in \url https://github.com/robotology/gazebo-yarp-plugins/pull/338/files/e757d8a40d21eebf8c55db9e1c47bf397a134c43#r161473858
 * for further information.
 */
class yarp::dev::GazeboYarpContactLoadCellArrayDriver : public yarp::dev::IAnalogSensor,
                                                               public yarp::dev::DeviceDriver,
                                                               public yarp::dev::IPreciselyTimed
    {
        public:
        GazeboYarpContactLoadCellArrayDriver();
        virtual ~GazeboYarpContactLoadCellArrayDriver();

        /**
         * Update callback from Gazebo
         * Gets the contact information from the link contact sensor
         * Computes equivalent wrench at the link CG (location CG, rotation link origin)
         * Transforms this wrench to the link origin (same force, moment is transformed)
         * Maps the transformed wrench to contact normal forces at load cell locations
         */
        void onUpdate(const gazebo::common::UpdateInfo& /*_info*/);
    
        // Device Driver
        virtual bool open(yarp::os::Searchable& config);
        virtual bool close();
    
        // Analog sensor
        virtual int read(yarp::sig::Vector &out);
        
        // Analog sensor unimplemented - not required in simulation
        virtual int getState(int ch);
        virtual int getChannels();
        virtual int calibrateChannel(int ch);
        virtual int calibrateChannel(int ch, double v);
        virtual int calibrateSensor();
        virtual int calibrateSensor(const yarp::sig::Vector& value);
    
        // Precisely Timed
        virtual yarp::os::Stamp getLastInputStamp(); 
    
        private:
       /**
        * Gets the link specified in the config file from the model
        * instance loaded by Gazebo and stores it as a gazebo::physics::LinkPtr 
        * \return true if specified link is found successfully 
        */
        bool initLinkAssociatedToContactSensor(yarp::os::Property &pluginParameters);
        
       /**
        * Parses the sdf for link associated to the contact sensor
        * to obtain the sensor name and the collision name of the link
        * Uses the Gazebo Sensor Manager to get the pointer to the sensor 
        * \return true if pointer to the contact sensor is set successfully
        */
        bool initContactSensor();
        
       /**
        * Reads the configuration file for load cell locations
        * \return true if successful
        */
        bool configure(yarp::os::Property &pluginParameters);
        
       /**
        * Prepares a matrix as a function of loadcell locations
        * which then maps an equivalent wrench to contact normal forces
        * at the load cell locations
        * \return true if successful
        */
        bool prepareMappingMatrix();
        
       /**
        * Gets position of frame associated to the link CG
        * with respect to the frame associated to the link origin
        */
        bool prepareLinkInformation();
                
        
        /**
         * Computes difference between measured CoP from equivalent wrench
         * and estimated CoP from the contact normal forces, assuming
         * a flat contact parallel to ground plane is made 
         * and the loadcell locations lie in the same XY plane
         */
        void checkCoP(const yarp::sig::Vector &threeAxisContactForceTorque);
        
        // Pointer to the contact sensor
        gazebo::sensors::ContactSensor* m_sensor;
        
        // Pointer to the robot model
        gazebo::physics::Model* m_robot;
        
        // Pointer to the link associated to the sensor
        gazebo::physics::Link* m_sensorLink;
        
        // Name of the link associated to the sensor
        std::string m_linkAssociateToSensor;
        
        // Name of the link collision associated to the sensor
        std::string m_linkCollisionName;
        
        // Position of link CG wrt link origin
        ignition::math::Vector3d m_linkOrigin_Pos_linkCG;
   
        // Timestamp
        yarp::os::Stamp m_stamp;
    
        // Mutex
        yarp::os::Mutex m_dataMutex;
        
        // Vector of magnitudes of contact normal forces acting at load cell locations
        yarp::sig::Vector m_contactNormalForces;
        
        // Vector of load cell locations with respect to the link origin
        std::vector<yarp::sig::Vector> m_loadCellLocations;
        
        // Matrix to map an equivalent wrench acting at link origin to contact normal forces at load cell locations
        yarp::sig::Matrix m_mapWrenchtoNormalForce;
        
        // Pointer to a connection signal to be tiggered when the model/sensor is updated
        gazebo::event::ConnectionPtr m_updateConnection;
    
        // Flag to indicate availability of the sensor output after first computation
        bool m_dataAvailable = false;
  };


#endif 
