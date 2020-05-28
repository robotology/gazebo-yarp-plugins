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
// available joint types in SDF (http://sdformat.org/spec?ver=1.6&elem=joint#joint_type)
// with the exception of gearbox joint
const std::vector<std::string> jointTypes { "revolute", "revolute2",
                                            "prismatic", "ball", "screw",
                                            "universal", "fixed"};

class LinkAttacherServerImpl: public GazeboYarpPlugins::LinkAttacherServer
{
private:
  gazebo::physics::WorldPtr _world;
  gazebo::physics::ModelPtr _model;
  std::string jointType;

public:
  LinkAttacherServerImpl();
  ~LinkAttacherServerImpl();

  /**
  * Attach any link of the models spawned in gazebo to a link of the robot using a fixed joint.
  * @param parent_model_name name that identifies the first model
  * @param parent_model_link_name name of the link in the first model you want to attach (scoped or unscoped name)
  * @param child_model_name name that identifies the second model
  * @param child_model_link_name name of the link in the second model you want to attach (scoped or unscoped name)
  * @return true if success, false otherwise
  */
  virtual bool attachUnscoped(const std::string& parent_model_name, const std::string& parent_model_link_name, const std::string& child_model_name, const std::string& child_model_link_name);

  /**
  * Detach the model link which was previously attached to the robot link through a fixed joint.
  * @param model_name name that identifies parent model
  * @param model_link_name name of the link of the parent model that is attached
  * @return true if success, false otherwise
  */
  virtual bool detachUnscoped(const std::string& model_name, const std::string& model_link_name);

  /**
  * Enable/disables gravity for a model
  * @param model_name name that identifies model in gazebo
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

  bool setJointType(const std::string j);

};

#endif
