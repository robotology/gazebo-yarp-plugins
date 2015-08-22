#ifndef YARPGAZEBO_WORLD_INTERFACESERVERIMPL
#define YARPGAZEBO_WORLD_INTERFACESERVERIMPL

#include <WorldInterfaceServer.h>

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include "worldproxy.h"

class WorldInterfaceServerImpl: public GazeboYarpPlugins::WorldInterfaceServer
{
private:
  WorldProxy *proxy;
   
public:
  WorldInterfaceServerImpl();
  ~WorldInterfaceServerImpl();
  
  virtual std::string makeSphere(const double x, const double y, const double z, const double radius, const int8_t r, const int8_t g, const int8_t b);
  virtual std::string makeBox(const double x, const double y, const double z, const double lx, const double ly, const double lz, const int8_t r, const int8_t g, const int8_t b);
  virtual std::string makeCyl(const double x, const double y, const double z, const double l, const double radius, const int8_t r, const int8_t g, const int8_t b);
  virtual bool setPosition(const std::string& id, double x, double y, double z);
  virtual std::vector<double>  getPosition(const std::string& id);
  virtual bool deleteAll();
  virtual std::vector<std::string>  getList();
  
  void attachWorldProxy(WorldProxy *p)
  {
    proxy=p;    
  } 
};

#endif
