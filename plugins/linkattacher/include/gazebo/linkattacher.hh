/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef YARPGAZEBO_LINKATTACHER
#define YARPGAZEBO_LINKATTACHER

#include <linkattacherserverimpl.h>
#include <memory>

namespace gazebo
{
  class LinkAttacher : public ModelPlugin
  {
  private:
    
     std::unique_ptr<yarp::os::Network> m_network;
     std::unique_ptr<yarp::os::RpcServer> m_rpcport;
     yarp::os::Property m_parameters;

     LinkAttacherServerImpl m_la_server;

   public:

      LinkAttacher();
     ~LinkAttacher();

      void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

  };

  //Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(LinkAttacher)
}

#endif
