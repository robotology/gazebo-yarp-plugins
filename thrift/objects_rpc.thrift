#objects plugin RPC server
namespace yarp gazebo

service ObjectsServer {

    /**
     * Create a sphere in simulation
     * @return true if success, false otherwise
     */
    bool createSphere(1:string name, 2:double radius, 3:double mass);

    /**
     * Get attach an object to a link of the robot.
     * @param link_name Name of the link
     * @param object_name Name of the object
     * @return true if success, false otherwise
     */
    bool attach(1:string link_name, 2:string object_name);

}
