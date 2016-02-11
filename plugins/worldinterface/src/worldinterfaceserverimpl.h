// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Lorenzo Natale
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 *
 */

#ifndef YARPGAZEBO_WORLD_INTERFACESERVERIMPL
#define YARPGAZEBO_WORLD_INTERFACESERVERIMPL

#include <WorldInterfaceServer.h>

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include "worldproxy.h"

class WorldInterfaceServerImpl: public GazeboYarpPlugins::WorldInterfaceServer
{
private:
  WorldProxy *proxy;

public:
  WorldInterfaceServerImpl();
  ~WorldInterfaceServerImpl();

 /**
   * Make a shpere.
   * @param radius radius of the sphere [m]
   * @param pose pose of the sphere [m]
   * @param color color of the sphere
   * @return returns a string that contains the name of the object in the world
   */
  virtual std::string makeSphere(const double radius, const GazeboYarpPlugins::Pose& pose, const GazeboYarpPlugins::Color& color);
  /**
   * Make a shpere.
   * @param width box width [m]
   * @param height box height[m]
   * @param thickness box thickness [m]
   * @param pose pose of the box [m]
   * @param color color of the box
   * @return returns a string that contains the name of the object in the world
   */
  virtual std::string makeBox(const double width, const double height, const double thickness, const GazeboYarpPlugins::Pose& pose, const GazeboYarpPlugins::Color& color);
  /**
   * Make a cylinder.
   * @param radius radius of the cylinder [m]
   * @param length lenght of the cylinder [m]
   * @param pose pose of the cylinder [m]
   * @param color color of the cylinder
   * @return returns a string that contains the name of the object in the world
   */
  virtual std::string makeCylinder(const double radius, const double length, const GazeboYarpPlugins::Pose& pose, const GazeboYarpPlugins::Color& color);
  /**
   * Set new object pose.
   * @param id object id
   * @param pose new pose
   * @return returns true or false on success failure
   */
  virtual bool setPose(const std::string& id, const GazeboYarpPlugins::Pose& pose);
  /**
   * Get object pose.
   * @param id string that identifies object in gazebo (returned after creation)
   * @return returns value of the pose
   */
  virtual GazeboYarpPlugins::Pose getPose(const std::string& id);

  /**
   * Enable/disables gravity for an object
   * @param id object id
   * @param enable n1 to enable gravity, 0 otherwise
   * @return returns true or false on success failure
   */
  virtual bool enableGravity(const std::string& id, const bool enable);
  
  virtual bool loadModelFromFile(const std::string& filename);


  /**
   * Delete all objects in the world.
   */
  virtual bool deleteAll();
  /**
   * List id of all objects that have been added to the world.
   * @return return a list of string containing the id of the objects
   */
  virtual std::vector<std::string>  getList();


  void attachWorldProxy(WorldProxy *p)
  {
    proxy=p;
  }
};

#endif
