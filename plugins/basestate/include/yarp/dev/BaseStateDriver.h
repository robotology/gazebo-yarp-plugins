/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#ifndef GAZEBOYARP_BASESTATEDRIVER_H
#define GAZEBOYARP_BASESTATEDRIVER_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>

#include <gazebo/physics/Link.hh>
#include <gazebo/physics/physics.hh>

#if GAZEBO_MAJOR_VERSION >= 8
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Matrix3.hh>
#include <ignition/math.hh>
#else 
#include <gazebo/math/Pose.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/math/Matrix3.hh>
#endif

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PreciselyTimed.h>

#include <yarp/os/Stamp.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/LockGuard.h>

#include <yarp/sig/Vector.h>

#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>


namespace yarp
{
    namespace dev
    {
        class GazeboYarpBaseStateDriver;
    }
}

/**
 * YARP device driver to give the gazebo state information of a desired base link of the robot
 * The state information include 
 * - absolute pose of the base link (x, y, z. roll, pitch, yaw)
 * - linear and angular velocity of the base link in the world frame
 * - linear and angular acceleration of the base link in the world frame
 * 
 * This device can be used by adding the following line in the SDF file of your robot,
 *  ```
 *      <plugin name="basestate" filename="libgazebo_yarp_basestate.so">
 *          <yarpConfigurationFile>model://path-to-the-configuration-file</yarpConfigurationFile>
 *      </plugin>
 *  ```
 * 
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
 * | device        |    string     |  -  |    -        |   Yes   | name of the device driver to instantiate               | MUST be set to gazebo_basestate           |
 * | baseLink      |    string     |  -  |    -        |   Yes   | name of the floating base link                         |                            -                     |
 *
 * An example configuration file might look like,
 * 
 * ```
 * [WRAPPER]
 * name /icubSim/floating_base/state:o
 * period 16
 * device analogServer
 * 
 * [DRIVER]
 * device gazebo_basestate
 * baseLink torso_link
 * ```
 */


class yarp::dev::GazeboYarpBaseStateDriver : public yarp::dev::IAnalogSensor,
                                                    public yarp::dev::DeviceDriver,
                                                    public yarp::dev::IPreciselyTimed
{
        public:
        GazeboYarpBaseStateDriver();
        virtual ~GazeboYarpBaseStateDriver();
        
        
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
    
        /**
         * Gets World pose, velocity and acceleration of the base link
         * and updates the base state vector
         */
        void onUpdate(const gazebo::common::UpdateInfo& _info);
    
        private:
            
        gazebo::physics::Model* m_robot;                      ///< Pointer to the loaded robot model
        gazebo::physics::LinkPtr m_baseLink;                  ///< Pointer to the desired base link from the model
        std::string m_baseLinkName;                           ///< Base link name
        const int m_stateDimensions = 18;                     ///< State dimensions to include 6D Pose, 6D velocity and 6D acceleration 
        yarp::sig::Vector m_baseState;                        ///< Vector for the base state
        yarp::os::Stamp m_stamp;                              ///< Current timestamp
        yarp::os::Mutex m_dataMutex;                          ///< Mutex for resource sharing 
        bool m_dataAvailable = false;                         ///< flag to check data is available
        gazebo::event::ConnectionPtr m_updateConnection;      ///< Event Pointer to the callback for updating the Gazebo information
               
};
#endif 

