#objects plugin RPC server
namespace yarp gazebo

service ObjectsServer {

    /**
     * Get attach an object to a link of the robot.
     * @param link_name Name of the link
     * @param object_name Name of the object
     * @return true if success, false otherwise
     */
    bool attach(1:string link_name, 2:string object_name);

    /**
     * Detaches an object to a link of the robot.
     * @param object_name Name of the object
     * @return true if success, false otherwise
     */
    bool detach(1:string object_name);

}
