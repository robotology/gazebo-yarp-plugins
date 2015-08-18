#ifndef YARPGAZEBO_WORLD_INTERFACESERVERIMPL
#define YARPGAZEBO_WORLD_INTERFACESERVERIMPL

#include <WorldInterfaceServer.h>

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

class WorldInterfaceServerImpl: public GazeboYarpPlugins::WorldInterfaceServer
{
private:
  
  class ObjectsList: public std::map<std::string, gazebo::physics::ModelPtr> 
  {  };
  typedef ObjectsList::iterator ObjectsListIt;
  typedef ObjectsList::const_iterator ObjectsListConstIt;


  gazebo::physics::WorldPtr world;
  gazebo::physics::ModelPtr model;
  ObjectsList objects;
  
public:
  virtual std::string makeSphere(const double x, const double y, const double z, const double radius, const int8_t r, const int8_t g, const int8_t b);
  virtual std::string makeBox(const double x, const double y, const double z, const double lx, const double ly, const double lz, const int8_t r, const int8_t g, const int8_t b);
  virtual std::string makeCyl(const double x, const double y, const double z, const double l, const double radius, const int8_t r, const int8_t g, const int8_t b);
  virtual bool setPosition(const std::string& id, const std::vector<double> & pos);
  virtual std::vector<double>  getPosition(const std::string& id);
  virtual bool deleteAll();
  virtual std::vector<std::string>  getList();
  
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
