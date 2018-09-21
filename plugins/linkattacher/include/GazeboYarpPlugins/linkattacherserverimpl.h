/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef YARPGAZEBO_LINKATTACHERSERVERIMPL
#define YARPGAZEBO_LINKATTACHERSERVERIMPL

#include "gazebo/gazebo.hh"
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/common/Events.hh>

#include <yarp/os/Network.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Property.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <LinkAttacherServer.h>

const std::string LogPrefix = "LinkAttacher:";

class LinkAttacherServerImpl: public GazeboYarpPlugins::LinkAttacherServer
{
private:
  gazebo::physics::WorldPtr _world;
  gazebo::physics::ModelPtr _model;

public:
  LinkAttacherServerImpl();
  ~LinkAttacherServerImpl();

  /**
  * Attach any link of the models spawned in gazebo to a link of the robot using a fixed joint.
  * @param model_name name that identifies model in gazebo (that are already spawned in gazebo)
  * @param model_link_name name of a the link in the model you want to attach to the robot
  * @param robot_name name of the robot
  * @param robot_link_name name of the robot link to which you want to attached the model link
  * @return true if success, false otherwise
  */
  virtual bool attachUnscoped(const std::string& model_name, const std::string& model_link_name, const std::string& robot_name, const std::string& robot_link_name);

  /**
  * Detach the model link which was previously attached to the robot link through a fixed joint.
  * @param model_name name that identifies model in gazebo (that are already spawned in gazebo)
  * @param model_link_name name of a the link in the model that is attached to the robot
  * @return true if success, false otherwise
  */
  virtual bool detachUnscoped(const std::string& model_name, const std::string& model_link_name);

  /**
  * Enable/disables gravity for a model
  * @param model_name name that identifies model in gazebo (that are already spawned in gazebo)
  * @param enable 1 to enable gravity, 0 otherwise
  * @return returns true or false on success failure
  */
  virtual bool enableGravity(const std::string& model_name, const bool enable);

  void attachWorldPointer(gazebo::physics::WorldPtr p)
  {
    _world=p;
  }

  void attachModelPointer(gazebo::physics::ModelPtr p)
  {
    _model=p;
  }

};

#endif
