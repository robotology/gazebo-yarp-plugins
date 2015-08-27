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
1: byte r; /* red channel */
2: byte g; /* green channel */
3: byte b; /* blue channel */
}


service WorldInterfaceServer {

    /** 
    * Make a shpere.
    * @param radius radius of the sphere [m]
    * @param pose pose of the sphere [m]
    * @param color color of the sphere
    * @return returns a string that contains the name of the object in the world
    */ 
    string makeSphere (1: double radius, 2: Pose pose, 3: Color color);
    
    
    /** 
    * Make a shpere.
    * @param width box width [m]
    * @param height box height[m]
    * @param thickness box thickness [m]
    * @param pose pose of the box [m]
    * @param color color of the box
    * @return returns a string that contains the name of the object in the world
    */
    string makeBox (1: double width, 2: double height, 3: double thickness, 4: Pose pose, 5: Color color);
    
    /** 
    * Make a cylinder.
    * @param radius radius of the cylinder [m]
    * @param length lenght of the cylinder [m]
    * @param pose pose of the cylinder [m]
    * @param color color of the cylinder
    * @return returns a string that contains the name of the object in the world
    */
    string makeCylinder (1: double radius, 2: double length, 3: Pose pose, 4: Color color);
    
     /** 
    * Set new object pose.
    * @param id object id
    * @param pose new pose
    * @return returns true or false on success failure
    */
    bool setPose(1: string id, 2: Pose pose);
    
     /** 
    * Get object pose.
    * @param id string that identifies object in gazebo (returned after creation)
    * @return returns value of the pose
    */
    Pose getPose(1:string id);
    
    /**
    * Load a model from file.
    * @param id string that specifies the name of the model
    * @return returns true/false on success failure.
    */
    bool loadModelFromFile(1:string filename);
      
    /**
    * Delete all objects in the world.
    */
    bool deleteAll();
    
    /**
    * List id of all objects that have been added to the world.
    * @return return a list of string containing the id of the objects
    */
    list<string> getList();
    
}
