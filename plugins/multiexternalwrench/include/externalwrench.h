#ifndef YARPGAZEBO_EXTERNALWRENCH_H
#define YARPGAZEBO_EXTERNALWRENCH_H


#include <iostream>
#include <stdlib.h>
#include <ctime>
#include <stdio.h>
#include <string>
#include <memory>

#include "gazebo/gazebo.hh"
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <GazeboYarpPlugins/common.h>

#include <yarp/os/Network.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/math/Math.h>

#include <boost/lexical_cast.hpp>

using namespace gazebo; 

class ExternalWrench
{
private:
   
   static int count;
   float color[4];
   struct wrenchCommand
   {
        std::string link_name;
        gazebo::math::Vector3 force;
        gazebo::math::Vector3 torque;
        double duration;
   };
   
   std::unique_ptr<wrenchCommand> wrenchPtr{new wrenchCommand()};
   
   double tick;
   double tock;
   gazebo::math::Vector3 *force_;
   gazebo::math::Vector3 *torque_;
   
   bool model_has_link;
   physics::ModelPtr model;
   physics::LinkPtr link;
   physics::Link_V model_links;
   
   transport::NodePtr m_node;
   transport::PublisherPtr m_visPub;
   msgs::Visual m_visualMsg;
   
   event::ConnectionPtr updateConnection;
    
    
public:
    
    bool duration_done;
    
    ExternalWrench();
    ~ExternalWrench();
    
    bool setWrench(physics::ModelPtr&, yarp::os::Bottle&);
    bool getLink();
    void applyWrench();
    void setModel();
};


#endif