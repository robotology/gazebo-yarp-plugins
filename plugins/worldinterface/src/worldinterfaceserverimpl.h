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
    virtual std::string makeSphere(const double radius, const GazeboYarpPlugins::Pose& pose, const GazeboYarpPlugins::Color& color, const std::string& frame_name = "", const std::string& object_name = "",const bool gravity_enable = 0, const bool collision_enable = 1);
    /**
     * Make a shpere.
     * @param width box width [m]
     * @param height box height[m]
     * @param thickness box thickness [m]
     * @param pose pose of the box [m]
     * @param color color of the box
     * @return returns a string that contains the name of the object in the world
     */
    virtual std::string makeBox(const double width, const double height, const double thickness, const GazeboYarpPlugins::Pose& pose, const GazeboYarpPlugins::Color& color, const std::string& frame_name = "", const std::string& object_name = "",const bool gravity_enable = 0, const bool collision_enable = 1);
    /**
     * Make a cylinder.
     * @param radius radius of the cylinder [m]
     * @param length lenght of the cylinder [m]
     * @param pose pose of the cylinder [m]
     * @param color color of the cylinder
     * @return returns a string that contains the name of the object in the world
     */
    virtual std::string makeCylinder(const double radius, const double length, const GazeboYarpPlugins::Pose& pose, const GazeboYarpPlugins::Color& color, const std::string& frame_name = "", const std::string& object_name = "",const bool gravity_enable = 0, const bool collision_enable = 1);
    /**
     * Set new object pose.
     * @param id object id
     * @param pose new pose
     * @param frame_name (optional) is specified, the pose will be relative to the specified fully scoped frame (e.g. MODEL_ID::FRAME_ID). Otherwise, world it will be used.
     * @return returns true or false on success failure
     */
    virtual bool setPose(const std::string& id, const GazeboYarpPlugins::Pose& pose, const std::string& frame_name = "");
    /**
     * Get object pose.
     * @param id string that identifies object in gazebo (returned after creation)
     * @param frame_name (optional) is specified, the pose will be relative to the specified fully scoped frame (e.g. MODEL1::link). Otherwise, world it will be used.
     * @return returns value of the pose in the world reference frame
     */
    virtual GazeboYarpPlugins::Pose getPose(const std::string& id, const std::string& frame_name = "");

    /**
     * Enable/disables gravity for an object
     * @param id object id
     * @param enable 1 to enable gravity, 0 otherwise
     * @return returns true or false on success failure
     */
    virtual bool enableGravity(const std::string& id, const bool enable);

    /**
     * Enable/disables collision detection for an object
     * @param id object id
     * @param enable 1 to enable collision detection, 0 otherwise
     * @return returns true or false on success failure
     */
    virtual bool enableCollision(const std::string& id, const bool enable);

    virtual bool loadModelFromFile(const std::string& filename);

    /**
     * Make a reference frame.
     * @param size size of the frame [m]
     * @param pose pose of the frame [m]
     * @param color color of the frame
     * @return returns a string that contains the name of the object in the world
     */
    virtual std::string makeFrame(const double size, const GazeboYarpPlugins::Pose& pose, const GazeboYarpPlugins::Color& color, const std::string& frame_name = "", const std::string& object_name = "", const bool gravity_enable = 0, const bool collision_enable = 1);

    /**
     * Change the color of an object
     * @param id object id
     * @param color color of the frame
     * @return returns true or false on success failure
     */
    virtual bool changeColor(const std::string& id, const GazeboYarpPlugins::Color& color);

    /**
     * Delete an object.
     * @param id string that identifies object in gazebo (returned after creation)
     * @return returns true/false on success failure.
     */
    virtual bool deleteObject(const std::string& id);
    /**
     * Delete all objects in the world.
     */
    virtual bool deleteAll();
    /**
     * List id of all objects that have been added to the world.
     * @return return a list of string containing the id of the objects
     */
    virtual std::vector<std::string>  getList();

    /**
     * Get attach an object to a link of the robot.
     * @param id string that identifies object in gazebo (returned after creation)
     * @param link_name name of a link of the robot
     * @return true if success, false otherwise
     */
    virtual bool attach(const std::string& id, const std::string& link_name);

    /**
     * Detach a previously attached object.
     * @param id string that identifies object in gazebo (returned after creation)
     * @return true if success, false otherwise
     */
    virtual bool detach(const std::string& id);

    /**
     * Change the names of an object.
     * @param old_name string that identifies object in gazebo
     * @param new_name string that will be used as new name
     * @return true if success, false otherwise
     */
    virtual bool rename(const std::string& old_name, const std::string& new_name);
    
    void attachWorldProxy(WorldProxy *p)
    {
        proxy=p;
    }
};

#endif
