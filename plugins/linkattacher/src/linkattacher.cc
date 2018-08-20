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

  bool configuration_loaded = false;

  if( _sdf->HasElement("yarpConfigurationFile") )
  {
    std::string ini_file_name = _sdf->Get<std::string>("yarpConfigurationFile");
    std::string ini_file_path = gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);

    if(!ini_file_path.empty() && m_parameters.fromConfigFile(ini_file_path.c_str()))
    {
      yInfo() << LogPrefix << "Load: Found yarpConfigurationFile: loading from " << ini_file_path;
      configuration_loaded = true;
    }

    if(!configuration_loaded)
    {
      yError() << LogPrefix << "Load: Failed to load the configuration file";
      return;
    }

    std::string portname;
    if(m_parameters.check("name"))
    {
          portname = m_parameters.find("name").asString();
    }
    else
    {
      yError() << LogPrefix << "Load: name parameter missing in the configuration file";
      return;
    }

    m_rpcport = std::unique_ptr<yarp::os::RpcServer>(new yarp::os::RpcServer());
    if(!m_rpcport->open(portname))
    {
      yError() << LogPrefix << "Load: failed to open rpcserver port";
      return;
    }

    m_la_server.yarp().attachAsServer(*m_rpcport);
  }

}
