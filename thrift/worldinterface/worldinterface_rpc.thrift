# Copyright (C) 2015 iCub Facility
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

#world interface RPC server
namespace yarp GazeboYarpPlugins

struct Pose {
1: double x;     /* x position [m] */
2: double y;     /* y position [m] */
3: double z;     /* z position [m] */
4: double roll;  /* rotation along roll axis, [rad] */
5: double pitch; /* rotation along pitch axis [rad]*/
6: double yaw;   /* rotation along yaw axis [rad]*/
}

struct Color {
1: i16 r; /* red channel in the range [0-255] */
2: i16 g; /* green channel in the range [0-255] */
3: i16 b; /* blue channel in the range [0-255] */
}


service WorldInterfaceServer {

    /** 
    * Make a sphere.
    * @param radius radius of the sphere [m]
    * @param pose pose of the sphere [m]
    * @param color color of the sphere
    * @param frame_name (optional) is specified, the pose will be relative to the specified fully scoped frame (e.g. MODEL_ID::FRAME_ID). Otherwise, world it will be used. 
    * @param object_name (optional) assigns a name to the object. 
    * @param gravity_enable (optional) enables gravity (default false)
    * @param collision_enable (optional) enables collision (default true)
    * @return returns a string that contains the name of the object in the world
    */ 
    string makeSphere (1: double radius, 2: Pose pose, 3: Color color, 4: string frame_name="", 5: string object_name="", 6: bool gravity_enable=0, 7: bool collision_enable=1);
    
    
    /** 
    * Make a box.
    * @param width box width [m]
    * @param height box height[m]
    * @param thickness box thickness [m]
    * @param pose pose of the box [m]
    * @param color color of the box
    * @param frame_name (optional) is specified, the pose will be relative to the specified fully scoped frame (e.g. MODEL_ID::FRAME_ID). Otherwise, world it will be used. 
    * @param object_name (optional) assigns a name to the object. 
    * @param gravity_enable (optional) enables gravity (default false)
    * @param collision_enable (optional) enables collision (default true)
    * @return returns a string that contains the name of the object in the world
    */
    string makeBox (1: double width, 2: double height, 3: double thickness, 4: Pose pose, 5: Color color, 6: string frame_name="", 7: string object_name="", 8: bool gravity_enable=0, 9: bool collision_enable=1);
    
    /** 
    * Make a cylinder.
    * @param radius radius of the cylinder [m]
    * @param length lenght of the cylinder [m]
    * @param pose pose of the cylinder [m]
    * @param color color of the cylinder
    * @param frame_name (optional) is specified, the pose will be relative to the specified fully scoped frame (e.g. MODEL_ID::FRAME_ID). Otherwise, world it will be used. 
    * @param object_name (optional) assigns a name to the object. 
    * @param gravity_enable (optional) enables gravity (default false)
    * @param collision_enable (optional) enables collision (default true)
    * @return returns a string that contains the name of the object in the world
    */
    string makeCylinder (1: double radius, 2: double length, 3: Pose pose, 4: Color color, 5: string frame_name="", 6: string object_name="", 7: bool gravity_enable=0, 8: bool collision_enable=1);
    
    /** 
    * Make a reference frame.
    * @param size size of the frame [m]
    * @param pose pose of the frame [m]
    * @param color color of the frame
    * @param frame_name (optional) is specified, the pose will be relative to the specified fully scoped frame (e.g. MODEL_ID::FRAME_ID). Otherwise, world it will be used. 
    * @param object_name (optional) assigns a name to the object. 
    * @param gravity_enable (optional) enables gravity (default false)
    * @param collision_enable (optional) enables collision (default true)
    * @return returns a string that contains the name of the object in the world
    */
    string makeFrame (1: double size, 2: Pose pose, 3: Color color, 4: string frame_name="", 5: string object_name="", 6: bool gravity_enable=0, 7: bool collision_enable=1);
    
    /** 
    * Change the color of an object
    * @param id object id
    * @param color color of the frame
    * @return returns true or false on success failure
    */
    bool changeColor (1: string id, 2: Color color);
    
     /** 
    * Set new object pose.
    * @param id object id
    * @param pose new pose
    * @param frame_name (optional) is specified, the pose will be relative to the specified fully scoped frame (e.g. MODEL_ID::FRAME_ID). Otherwise, world it will be used.
    * @return returns true or false on success failure
    */
    bool setPose(1: string id, 2: Pose pose, 3: string frame_name="");
    
     /** 
    * Enable/disables gravity for an object
    * @param id object id
    * @param enable 1 to enable gravity, 0 otherwise
    * @return returns true or false on success failure
    */
    bool enableGravity (1: string id, 2: bool enable);

     /** 
    * Enable/disables collision detection for an object
    * @param id object id
    * @param enable 1 to enable collision detection, 0 otherwise
    * @return returns true or false on success failure
    */
    bool enableCollision (1: string id, 2: bool enable);
    
     /** 
    * Get object pose.
    * @param id string that identifies object in gazebo (returned after creation)
    * @param frame_name (optional) is specified, the pose will be relative to the specified fully scoped frame (e.g. MODEL_ID::FRAME_ID). Otherwise, world it will be used.
    * @return returns value of the pose in the world reference frame
    */
    Pose getPose(1:string id, 2: string frame_name="");
    
    /**
    * Load a model from file.
    * @param id string that specifies the name of the model
    * @return returns true/false on success failure.
    */
    bool loadModelFromFile(1:string filename);
    
     /** 
    * Delete an object.
    * @param id string that identifies object in gazebo (returned after creation)
    * @return returns true/false on success failure.
    */
    bool deleteObject(1:string id);
  
    /**
    * Delete all objects in the world.
    */
    bool deleteAll();
    
    /**
    * List id of all objects that have been added to the world.
    * @return return a list of string containing the id of the objects
    */
    list<string> getList();
    
    /**
    * Attach an object to a link of the robot.
    * @param id string that identifies object in gazebo (returned after creation)
    * @param link_name name of a link of the robot
    * @return true if success, false otherwise
    */
    bool attach(1:string id, 2:string link_name);
    
    /**
    * Detach a previously attached object.
    * @param id string that identifies object in gazebo (returned after creation)
    * @return true if success, false otherwise
    */
    bool detach(1:string id);
    
    /**
    * Change the names of an object.
    * @param old_name string that identifies object in gazebo 
    * @param new_name string that will be used as new name
    * @return true if success, false otherwise
    */
    bool rename(1:string old_name, 2:string new_name);
}
