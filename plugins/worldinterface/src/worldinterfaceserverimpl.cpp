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

std::string WorldInterfaceServerImpl::makeSphere(const double radius, const GazeboYarpPlugins::Pose& pose, const GazeboYarpPlugins::Color& color)
{
  return proxy->makeSphere(radius, pose, color);
  
}

std::string WorldInterfaceServerImpl::makeBox(const double width, const double height, const double thickness, const GazeboYarpPlugins::Pose& pose, const GazeboYarpPlugins::Color& color)
{
  return proxy->makeBox(width, height, thickness, pose, color);
}

std::string WorldInterfaceServerImpl::makeCylinder(const double radius, const double length, const GazeboYarpPlugins::Pose& pose, const GazeboYarpPlugins::Color& color)
{
  return proxy->makeCylinder(radius, length, pose, color);
}

bool WorldInterfaceServerImpl::setPose(const std::string& id, const GazeboYarpPlugins::Pose& pose)
{
  return proxy->setPose(id, pose);
}

GazeboYarpPlugins::Pose WorldInterfaceServerImpl::getPose(const std::string& id)
{
  return proxy->getPose(id); 
}
  
bool WorldInterfaceServerImpl::loadModelFromFile(const std::string& filename)
{
  return proxy->loadModelFromFile(filename);  
}

  
bool WorldInterfaceServerImpl::deleteAll()
{
  return proxy->deleteAll();
}

std::vector<std::string> WorldInterfaceServerImpl::getList()
{
  return proxy->getList();
}


  
