/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef __GAZEBO_YARP_FORCETORQUE_PLUGIN_HH__
#define __GAZEBO_YARP_FORCETORQUE_PLUGIN_HH__

#include "gazebo/sensors/sensors.hh"
#include "gazebo/plugins/ForceTorquePlugin.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/math/Pose.hh"
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>
#include <analogServer.h>

#include <fakebotFTsensor.h>


namespace gazebo
{
  /// \brief An base class plugin for custom force torque sensor processing.
  class GazeboYarpForceTorque : public ForceTorquePlugin
  {
    /// \brief Constructor
    public: GazeboYarpForceTorque();

    /// \brief Destructor
    public: virtual ~GazeboYarpForceTorque();

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

    /// \brief pointer to current world
    private: gazebo::physics::WorldPtr world;

    /// \brief joint providing the force/torque measurements
    private: physics::LinkPtr ftLink;

    /// \brief Pointer to the YARP FT sensor device driver
    private: yarp::dev::GazeboYarpForceTorqueDriver* yarpFTsensor;
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
