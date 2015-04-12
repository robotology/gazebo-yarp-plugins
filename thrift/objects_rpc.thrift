#objects plugin RPC server
namespace yarp gazebo

service ObjectsServer {

    /**
     * Get attach an object to a link of the robot.
     * @param link_name Name of the link
     * @param object_name Name of the box to be operated
     * @param width width of the box to be operated
     * @param height height of the box to be operated
     * @param length length of the box to be operated
     * @return true if success, false otherwise
     */
    bool attach(1:string link_name, 2:string object_name, 3:double width, 4:double height, 5:double length);


    /**
     * Detaches an object to a link of the robot.
     * @param object_name Name of the object
     * @return true if success, false otherwise
     */
    bool detach(1:string object_name);

    /**
     * Delete an object from Gazebo environment.
     * @param object_name Name of the object
     * @return true if success, false otherwise
     */
    bool deleteObject(1:string object_name);

}
