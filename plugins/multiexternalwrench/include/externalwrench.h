#ifndef YARPGAZEBO_EXTERNALWRENCH_H
#define YARPGAZEBO_EXTERNALWRENCH_H


#include <iostream>
#include <string>

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

using namespace gazebo; 

class ExternalWrench: public yarp::os::Thread
{
private:

   struct wrenchCommand
   {
        std::string link_name;
        math::Vector3 force;
        math::Vector3 torque;
        double duration;
   };
   
   wrenchCommand *wrench;
   double tick;
   double tock;
   
   physics::ModelPtr model;
   physics::LinkPtr link;
   physics::Link_V model_links;
   
   event::ConnectionPtr updateConnection;
    
    
public:
    
    
    virtual bool threadInit();
    virtual void run();
    virtual void threadRelease(); 
    
    bool setWrench(physics::ModelPtr&, yarp::os::Bottle&);
    bool getLink();
    void applyWrench();
    void setModel();
};


#endif