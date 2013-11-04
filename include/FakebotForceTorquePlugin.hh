/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef GAZEBO_FAKEBOT_FT_PLUGIN_HH
#define GAZEBO_FAKEBOT_FT_PLUGIN_HH

#include "gazebo/sensors/sensors.hh"
#include "gazebo/plugins/ForceTorquePlugin.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>
#include <analogServer.h>

#include <fakebotFTsensor.h>


namespace gazebo
{
  /// \brief An base class plugin for custom force torque sensor processing.
  class FakebotForceTorquePlugin : public ForceTorquePlugin
  {
    /// \brief Constructor
    public: FakebotForceTorquePlugin();

    /// \brief Destructor
    public: virtual ~FakebotForceTorquePlugin();

    /// \brief Load the plugin.
    /// \param[in] _parent Pointer to the parent sensor.
    /// \param[in] _sdf SDF element for the plugin.
    public: virtual void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update callback. Overload this function in a child class.
    /// \param[in] _msg The force torque message.
    protected: virtual void OnUpdate(msgs::WrenchStamped _msg);

    /// \brief The parent sensor
    protected: sensors::ForceTorqueSensorPtr parentSensor;

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr connection;

    /// \brief Pointer to the YARP FT sensor device driver
    private: yarp::dev::fakebotFTsensor* yarpFTsensor;
    private: yarp::dev::PolyDriver _driver;

    /// \brief we have one analogserver for each ft sensor
    private: yarp::dev::AnalogServer *_server;

    private: yarp::os::Network _yarp;


    /// \brief static unsigned int kept fo always have a unique FT sensor id
    private: static unsigned int iBoards;
    /// \brief FT sensor id;the measurements from the FT sensors are sorted by id
    private: unsigned int boardId;
  };
}
#endif
