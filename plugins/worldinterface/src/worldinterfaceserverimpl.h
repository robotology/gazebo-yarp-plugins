#ifndef YARPGAZEBO_WORLD_INTERFACESERVERIMPL
#define YARPGAZEBO_WORLD_INTERFACESERVERIMPL

#include <WorldInterfaceServer.h>

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

class WorldInterfaceServerImpl: public GazeboYarpPlugins::WorldInterfaceServer
{
private:
  gazebo::physics::WorldPtr world;
  gazebo::physics::ModelPtr model;
public:
  bool addBall();
  
  void attachWorldPointer(gazebo::physics::WorldPtr p)
  {
    world=p;
  }
  
  void attachModelPointer(gazebo::physics::ModelPtr p)
  {
    model=p;
  }
};

#endif
