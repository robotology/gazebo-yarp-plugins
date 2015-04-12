/*
 * Copyright (C) 2007-2015 Istituto Italiano di Tecnologia. RBCS Department.
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "thrift/ObjectsServer.h"

namespace gazebo {

    class GazeboYarpObjects;

class ObjectsServerImpl : public ObjectsServer {
public:

    ObjectsServerImpl(GazeboYarpObjects& objectsPlugin);
    /**
    * Get attach an object to a link of the robot.
    * @param link_name Name of the link
    * @param object_name Name of the object
    * @return true if success, false otherwise
    */
    //virtual bool attach(const std::string& link_name, const std::string& object_name);
    virtual bool attach(const std::string& link_name, const std::string& object_name, const double width, const double height, const double length);

    /**
     * Detach an object from the links it was attached to
     * @param object_name Name of the object
     * @return true if success, false otherwise
     */
    virtual bool detach(const std::string& object_name);

    /**
     * Delete an object from the Gazebo environment
     * @param object_name Name of the object
     * @return true if success, false otherwise
     */
    virtual bool deleteObject(const std::string& object_name);
private:

    GazeboYarpObjects& m_objects;
};

}
