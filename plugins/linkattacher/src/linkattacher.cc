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

  if(m_parameters.check("STARTUP"))
  {
      yInfo() << LogPrefix << "Applying commands at startup";

      yarp::os::Bottle startupConfig = m_parameters.findGroup("STARTUP");

      for(size_t i=1; i<startupConfig.size(); i++)
      {
          if(startupConfig.get(i).check("attachUnscoped") && startupConfig.get(i).find("attachUnscoped").isList() && startupConfig.get(i).find("attachUnscoped").asList()->size()==4)
          {
              yarp::os::Bottle* attachConfigList = startupConfig.get(i).find("attachUnscoped").asList();
              m_la_server.attachUnscoped(attachConfigList->get(0).asString(), attachConfigList->get(1).asString(), attachConfigList->get(2).asString(), attachConfigList->get(3).asString());
          }
          else if(startupConfig.get(i).check("detachUnscoped") && startupConfig.get(i).find("detachUnscoped").isList() && startupConfig.get(i).find("detachUnscoped").asList()->size()==2)
          {
              yarp::os::Bottle* detachConfigList = startupConfig.get(i).find("detachUnscoped").asList();
              m_la_server.detachUnscoped(detachConfigList->get(0).asString(), detachConfigList->get(1).asString());
          }
          else if(startupConfig.get(i).check("enableGravity") && startupConfig.get(i).find("enableGravity").isList() && startupConfig.get(i).find("enableGravity").asList()->size()==2)
          {
              yarp::os::Bottle* enableGravityConfigList = startupConfig.get(i).find("enableGravity").asList();
              m_la_server.enableGravity(enableGravityConfigList->get(0).asString(), enableGravityConfigList->get(1).asBool());
          }
          else
          {
              yWarning() << LogPrefix << "Failed to load startup configuration line [" << i << "]";
          }
      }
  }

  m_la_server.yarp().attachAsServer(*m_rpcport);
}
