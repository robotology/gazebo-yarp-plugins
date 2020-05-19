/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include <linkattacher.hh>

using namespace gazebo;
using namespace std;
using namespace yarp::os;

LinkAttacher::LinkAttacher()
{
}

LinkAttacher::~LinkAttacher()
{
  if(m_rpcport)
  {
    m_rpcport->close();
  }
}

void LinkAttacher::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  if(m_network != 0)
  {
    return;
  }

  m_network = std::unique_ptr<yarp::os::Network>(new yarp::os::Network());

  if(!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout))
  {
    yError() << LogPrefix << "Load: yarp network does not seem to be available, is the yarpserver running?";
    return;
  }

  std::string model_name = _model->GetName();

  yInfo() << LogPrefix << "Load: Model name: " << model_name;

  m_la_server.attachWorldPointer(_model->GetWorld());
  m_la_server.attachModelPointer(_model);

  // Getting .ini configuration file parameters from sdf
  bool configuration_loaded = GazeboYarpPlugins::loadConfigModelPlugin(_model, _sdf, m_parameters);

  if (!configuration_loaded)
  {
      yError() << LogPrefix << " File .ini not found, load failed." ;
      return;
  }

  std::string jointType;
  if(m_parameters.check("jointType"))
  {
      jointType = m_parameters.find("jointType").asString();
  }
  else
  {
      jointType="fixed";
  }

  if(!m_la_server.setJointType(jointType))
  {
      return;
  }
  
  std::string portname;
  if(m_parameters.check("name"))
  {
        portname = m_parameters.find("name").asString();
  }
  else
  {
    yError() << LogPrefix << " <name> parameter missing in the configuration file";
    return;
  }

  m_rpcport = std::unique_ptr<yarp::os::RpcServer>(new yarp::os::RpcServer());
  if(!m_rpcport->open(portname))
  {
    yError() << LogPrefix << " failed to open rpcserver port";
    return;
  }

  m_la_server.yarp().attachAsServer(*m_rpcport);
}
