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
     * Create a sphere in simulation
     * @return true if success, false otherwise
     */
    virtual bool createSphere(const std::string& name, const double radius, const double mass);
    /**
    * Get attach an object to a link of the robot.
    * @param link_name Name of the link
    * @param object_name Name of the object
    * @return true if success, false otherwise
    */
    virtual bool attach(const std::string& link_name, const std::string& object_name);

private:

    GazeboYarpObjects& m_objects;
};

}
