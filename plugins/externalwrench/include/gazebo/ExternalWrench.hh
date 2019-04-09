/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef GAZEBO_YARP_PLUGINS_EXTERNALWRENCH_H
#define GAZEBO_YARP_PLUGINS_EXTERNALWRENCH_H

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

using namespace gazebo;

class ExternalWrench
{
private:

   static int count;
   float      color[4];
   struct wrenchCommand
   {
        std::string              link_name;
        yarp::sig::Vector        force;
        yarp::sig::Vector        torque;
        double                   duration;
   };

   std::unique_ptr<wrenchCommand> wrenchPtr{new wrenchCommand()};

   double                        tick;
   double                        tock;

   physics::ModelPtr             model;
   physics::LinkPtr              link;

   transport::NodePtr            node;
   transport::PublisherPtr       visPub;
   msgs::Visual                  visualMsg;

   event::ConnectionPtr          updateConnection;

   void setVisual();

public:

    bool duration_done;

    ExternalWrench();
    ~ExternalWrench();

    bool setWrench(physics::ModelPtr&, yarp::os::Bottle&);
    bool getLink();

    void setTick(double& tickTime);
    void setTock(double& tockTime);
    void applyWrench();
    void deleteWrench();
    void setModel();
};

#endif //GAZEBO_YARP_PLUGINS_EXTERNALWRENCH_H
