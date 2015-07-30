#include "worldinterface.h"
#include <iostream>
#include <yarp/os/Network.h>

#include <GazeboYarpPlugins/common.h>

using namespace gazebo;
using namespace std;

WorldInterface::WorldInterface() : ModelPlugin(),
m_network(0),
m_rpcport(0)
{
    cout << "*** WorldInterface contructor ***" << endl;
}

WorldInterface::~WorldInterface()
{
  cout << "*** WorldInterface closing ***" << endl;
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
        cerr << "WorldInterface::Load error: yarp network does not seem to be available, is the yarpserver running?"<<std::endl;
        return;
  }
  
  m_wi_server.attachWorldPointer(_model->GetWorld());
  m_wi_server.attachModelPointer(_model);
  
  //Getting .ini configuration file from sdf
  bool configuration_loaded = false;
    
  if (_sdf->HasElement("yarpConfigurationFile")) {
        std::string ini_file_name = _sdf->Get<std::string>("yarpConfigurationFile");
        std::string ini_file_path = gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);
        
        if (ini_file_path != "" && m_parameters.fromConfigFile(ini_file_path.c_str())) {
            std::cout << "Found yarpConfigurationFile: loading from " << ini_file_path << std::endl;
            configuration_loaded = true;
        }
  }
  
  if (!configuration_loaded) {
    cerr << "WorldInterface::Load error could not load configuration file"<<std::endl;
    return;
  }
    
  std::string portname=m_parameters.find("name").asString();
    
  m_rpcport=new yarp::os::RpcServer();
  m_rpcport->open(portname);
  
  m_wi_server.yarp().attachAsServer(*m_rpcport);
  
}
