/*
* Copyright (C) 2007-2014 Istituto Italiano di Tecnologia RBCS, ADVR and iCub Facility
* Authors: Mirko Ferrati, Cheng Fang, Silvio Traversaro
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#ifndef GAZEBOYARP_OBJECTS_HH
#define GAZEBOYARP_OBJECTS_HH

#include <gazebo/common/Plugin.hh>
#include <yarp/os/Network.h>
#include <gazebo/physics/PhysicsTypes.hh>

namespace yarp {
    namespace os {
        class Port;
        class Bottle;
    }
}

namespace gazebo
{
    class ObjectsServer;

    class GazeboYarpObjects : public ModelPlugin
    {
    public:
        GazeboYarpObjects();

        virtual ~GazeboYarpObjects();

        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        void gazeboYarpObjectsLoad(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        virtual bool attach(const std::string& link_name, const std::string& object_name);

        virtual bool detach(const std::string& object_name);

    private:
        void cleanup();

        yarp::os::Network m_network;
        std::string m_portName;

        physics::ModelPtr m_model;
        physics::WorldPtr m_world;

        //std::map< std::string , physics::LinkPtr > objects_link_map;

        //RPC variables
        yarp::os::Port *m_rpcPort;
        ObjectsServer *m_clockServer;
        std::map<std::string, physics::JointPtr> joints_attached;

    };
}




#endif
