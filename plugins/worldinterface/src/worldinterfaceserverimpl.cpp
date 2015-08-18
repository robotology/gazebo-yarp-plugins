#include "worldinterfaceserverimpl.h"
#include <string>
#include <list>

using namespace gazebo;
using namespace GazeboYarpPlugins;
using namespace std;

void replace(std::string &str, std::string key, double value)
{
  std::ostringstream tmp;
  tmp << value;

  size_t pos = 0;
    while ((pos = str.find(key, pos)) != std::string::npos) {
         str.replace(pos, key.length(), tmp.str());
         pos += tmp.str().length();
    }
}
  
std::string WorldInterfaceServerImpl::makeSphere(const double x, const double y, const double z, const double radius, const int8_t r, const int8_t g, const int8_t b)
{
  sdf::SDF sphereSDF;
 
  string sphereSDF_string=string(
       "<sdf version ='1.4'>\
          <model name ='sphere'>\
            <pose>POSEX POSEY POSEZ 0 0 0</pose>\
            <link name ='link'>\
              <pose>POSEX POSEY POSEZ 0 0 0</pose>\
              <collision name ='collision'>\
                <geometry>\
                  <sphere><radius>RADIUS</radius></sphere>\
                </geometry>\
              </collision>\
              <visual name ='visual'>\
                <geometry>\
                  <sphere><radius>RADIUS</radius></sphere>\
                </geometry>\
              </visual>\
            </link>\
          </model>\
        </sdf>");

  replace(sphereSDF_string, "POSEX", x);
  replace(sphereSDF_string, "POSEY", y);
  replace(sphereSDF_string, "POSEZ", z);
  replace(sphereSDF_string, "RADIUS", radius);
  
  sphereSDF.SetFromString(sphereSDF_string);
  
  int nobjects=objects.size()+1;
  std::ostringstream objlabel;
  objlabel << "sphere"<< nobjects;
  
  sdf::ElementPtr model = sphereSDF.root->GetElement("model");

  model->GetAttribute("name")->SetFromString(objlabel.str());
  world->InsertModelSDF(sphereSDF);
  
  gazebo::physics::ModelPtr tmp=world->GetModel(objlabel.str());
  objects.insert(std::pair<string,gazebo::physics::ModelPtr>(objlabel.str(), tmp));
  
  return objlabel.str();
}

std::string WorldInterfaceServerImpl::makeBox(const double x, const double y, const double z, const double lx, const double ly, const double lz, const int8_t r, const int8_t g, const int8_t b)
{
  
  return std::string("");
}

std::string WorldInterfaceServerImpl::makeCyl(const double x, const double y, const double z, const double l, const double radius, const int8_t r, const int8_t g, const int8_t b)
{
  sdf::SDF cylSDF;
 
  string cylSDF_String=string(
    "<?xml version='1.0'?>\
	<sdf version ='1.4'>\
        <model name ='cylinder'>\
	  <pose>POSEX POSEY POSEZ  0 0 0</pose>\
	    <link name ='link'>\
      <pose>POSEX POSEY POSEZ 0 0 0</pose>\
      <collision name ='collision'>\
        <geometry>\
          <cylinder><radius>RADIUS</radius><length>LENGTH</length></cylinder>\
        </geometry>\
      </collision>\
      <visual name='visual'>\
        <geometry>\
          <cylinder><radius>RADIUS</radius><length>LENGTH</length></cylinder>\
	  </geometry>\
	</visual>\
	</link>\
      </model>\
  </sdf>");

  replace(cylSDF_String, "POSEX", x);
  replace(cylSDF_String, "POSEY", y);
  replace(cylSDF_String, "POSEZ", z);
  replace(cylSDF_String, "RADIUS", radius);
  replace(cylSDF_String, "LENGTH", l);
  
  cylSDF.SetFromString(cylSDF_String);
  
  int nobjects=objects.size()+1;
  std::ostringstream objlabel;
  objlabel << "cylinder"<<nobjects;
  
  sdf::ElementPtr model = cylSDF.root->GetElement("model");
  model->GetAttribute("name")->SetFromString(objlabel.str());
  world->InsertModelSDF(cylSDF);
 
  gazebo::physics::ModelPtr tmp=world->GetModel(objlabel.str());
  objects.insert(std::pair<string,physics::ModelPtr>(objlabel.str(), tmp));
  return objlabel.str();
}

bool WorldInterfaceServerImpl::setPosition(const std::string& id, const std::vector<double> & pos)
{
  if (pos.size()!=3)
  {
    cerr<<"Asking non valid position (size should be 3)\n";
    return false;
  }
  
  gazebo::physics::ModelPtr model=objects[id];
  
  if (!model)
  {
    cerr<<"Object " << id << " does not exist\n";
    return false;
  }
  
  math::Pose p(pos[0], pos[1], pos[2], 0, 0, 0);
  
  model->SetWorldPose(p);
  
  return true;
}


std::vector<double>  WorldInterfaceServerImpl::getPosition(const std::string& id)
{
  // check that id is valid
  vector<double> ret;
    
  gazebo::physics::ModelPtr model= objects[id];
  
  if (!model)
  {
    cerr<<"Object " << id << " does not exist\n";
    return ret;
  }
  
  math::Pose p=model->GetWorldPose();
  
  ret.push_back(p.pos[0]);
  ret.push_back(p.pos[1]);
  ret.push_back(p.pos[2]);

  return ret;
}

bool WorldInterfaceServerImpl::deleteAll()
{
  ObjectsListIt it=objects.begin();
  while(it!=objects.end())
  {
    world->RemoveModel(it->first);
    it++;
  }
  
  objects.clear();
  
  return true;
}

std::vector<std::string> WorldInterfaceServerImpl::getList()
{
  vector<std::string> ret;
  
  ObjectsListIt it=objects.begin();
  while(it!=objects.end())
  {
    ret.push_back(it->first);
    it++;
  }
  
  return ret;
}
  
