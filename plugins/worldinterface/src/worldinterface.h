#ifndef YARPGAZEBO_WORLD_INTERFACE
#define YARPGAZEBO_WORLD_INTERFACE


#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include "worldinterfaceserverimpl.h"
#include "worldproxy.h"

#include <yarp/os/Network.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Property.h>

namespace gazebo
{
class WorldInterface : public ModelPlugin
{
private:
  event::ConnectionPtr updateConnection;
  
  WorldInterfaceServerImpl m_wi_server;
  yarp::os::RpcServer *m_rpcport;
  yarp::os::Network   *m_network;
  yarp::os::Property m_parameters; 

  WorldProxy m_proxy;
  
     
  public: 
    WorldInterface();
    ~WorldInterface();
    
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
    void OnUpdate(const gazebo::common::UpdateInfo & /*_info*/);

    
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(WorldInterface)
}


#endif