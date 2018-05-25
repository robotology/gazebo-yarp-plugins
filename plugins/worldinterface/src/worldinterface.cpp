// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Lorenzo Natale
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 *
 */

 #include "worldinterface.h"
#include <iostream>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <GazeboYarpPlugins/common.h>

using namespace gazebo;
using namespace std;

WorldInterface::WorldInterface() : ModelPlugin(),
m_network(0),
m_rpcport(0)
{
}

WorldInterface::~WorldInterface()
{
  //chck deadlocks, maybe we need to add timeouts to worldproxy waiting functions
  if (m_rpcport)
  {
    m_rpcport->close();
    delete m_rpcport;
  }

  if (m_network)
    delete m_network;
}

void WorldInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  if (m_network!=0)
    return;

  m_network=new yarp::os::Network();

  if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
        yError() << "WorldInterface::Load error: yarp network does not seem to be available, is the yarpserver running?";
        return;
  }

  //setting up proxy
  m_proxy.attachWorldPointer(_model->GetWorld());
  m_proxy.attachModelPointer(_model);

  //pass reference to server
  m_wi_server.attachWorldProxy(&m_proxy);

  //Getting .ini configuration file from sdf
  bool configuration_loaded = false;

  if (_sdf->HasElement("yarpConfigurationFile")) {
        std::string ini_file_name = _sdf->Get<std::string>("yarpConfigurationFile");
        std::string ini_file_path = gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);

        if (ini_file_path != "" && m_parameters.fromConfigFile(ini_file_path.c_str())) {
            yInfo() << "Found yarpConfigurationFile: loading from " << ini_file_path ;
            configuration_loaded = true;
        }
  }

  if (!configuration_loaded) {
    yError() << "WorldInterface::Load error could not load configuration file";
    return;
  }

  std::string portname=m_parameters.find("name").asString();
  int synchronous=m_parameters.find("synchro").asInt32();

  if (synchronous)
     m_proxy.setSynchronousMode(true);
  else
     m_proxy.setSynchronousMode(false);

  m_rpcport=new yarp::os::RpcServer();
  m_rpcport->open(portname);

  m_wi_server.yarp().attachAsServer(*m_rpcport);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&WorldInterface::OnUpdate, this, _1));

}

void WorldInterface::OnUpdate(const common::UpdateInfo & _info)
{
  m_proxy.update(_info);
}
