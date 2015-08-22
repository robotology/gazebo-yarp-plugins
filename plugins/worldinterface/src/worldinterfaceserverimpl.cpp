#include "worldinterfaceserverimpl.h"
#include <string>
#include <list>

using namespace gazebo;
using namespace GazeboYarpPlugins;
using namespace std;

WorldInterfaceServerImpl::WorldInterfaceServerImpl()
{}

WorldInterfaceServerImpl::~WorldInterfaceServerImpl()
{}
  
std::string WorldInterfaceServerImpl::makeSphere(const double x, const double y, const double z, const double radius, const int8_t r, const int8_t g, const int8_t b)
{
  return proxy->makeSphere(x, y, z, radius, r, g, b);
}

std::string WorldInterfaceServerImpl::makeBox(const double x, const double y, const double z, const double lx, const double ly, const double lz, const int8_t r, const int8_t g, const int8_t b)
{
  
  return proxy->makeBox(z, y, z, lx, ly, lz, r, g, b);
}

std::string WorldInterfaceServerImpl::makeCyl(const double x, const double y, const double z, const double l, const double radius, const int8_t r, const int8_t g, const int8_t b)
{
  return proxy->makeCyl(x, y, z, l, radius, r, g, b);
}

bool WorldInterfaceServerImpl::setPosition(const std::string& id, double x, double y, double z)
{
  return proxy->setPosition(id, x, y, z);
}


std::vector<double>  WorldInterfaceServerImpl::getPosition(const std::string& id)
{
  return proxy->getPosition(id);
}

bool WorldInterfaceServerImpl::deleteAll()
{
  return proxy->deleteAll();
}

std::vector<std::string> WorldInterfaceServerImpl::getList()
{
  return proxy->getList();
}


  
