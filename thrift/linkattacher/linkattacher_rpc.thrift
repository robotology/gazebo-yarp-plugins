#link attacher RPC server
namespace yarp GazeboYarpPlugins

service LinkAttacherServer {

    /**
    * Enable/disables gravity for a model
    * @param model_name name that identifies model in gazebo (that are already spawned in gazebo)
    * @param enable 1 to enable gravity, 0 otherwise
    * @return returns true or false on success failure
    */
    bool enableGravity (1: string model_name, 2: bool enable);

    /**
    * Attach any link of the models spawned in gazebo to a link of the robot using a fixed joint.
    * @param model_name name that identifies model in gazebo (that are already spawned in gazebo)
    * @param model_link_name name of a the link in the model you want to attach to the robot
    * @param robot_name name of the robot
    * @param robot_link_name name of the robot link to which you want to attached the model link
    * @return true if success, false otherwise
    */
    bool attachUnscoped(1:string model_name, 2:string model_link_name, 3:string robot_name, 4:string robot_link_name);

    /**
    * Detach the model link which was previously attached to the robot link through a fixed joint.
    * @param model_name name that identifies model in gazebo (that are already spawned in gazebo)
    * @param model_link_name name of a the link in the model that is attached to the robot
    * @return true if success, false otherwise
    */
    bool detachUnscoped(1:string model_name, 2:string model_link_name);
}
