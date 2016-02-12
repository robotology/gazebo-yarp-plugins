// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Lorenzo Natale
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 *
 */

 #include "worldproxy.h"
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <math.h>

#include <string>
#include <iostream>
#include <ostream>

using namespace std;
using namespace gazebo;

#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

void replace(string &str, string key, double value)
{
  ostringstream tmp;
  tmp << value;

  size_t pos = 0;
    while ((pos = str.find(key, pos)) != string::npos) {
         str.replace(pos, key.length(), tmp.str());
         pos += tmp.str().length();
    }
}


WorldProxy::WorldProxy():
isSynchro(true)
{}

WorldProxy::~WorldProxy()
{}


sdf::ElementPtr getSDFRoot(sdf::SDF &sdfObj)
{
#if SDF_MAJOR_VERSION >= 3
  return sdfObj.Root();
#else
  return sdfObj.root;
#endif
}

std::string WorldProxy::makeSphere(const double radius, const GazeboYarpPlugins::Pose& pose, const GazeboYarpPlugins::Color& color)
{
  sdf::SDF sphereSDF;

  string sphereSDF_string=string(
      "<?xml version='1.0'?>\
       <sdf version ='1.4'>\
          <model name ='sphere'>\
            <pose>POSEX POSEY POSEZ ROLL PITCH YAW</pose>\
            <link name ='link'>\
              <pose>0 0 0 0 0 0</pose>\
              <collision name ='collision'>\
                <geometry>\
                  <sphere><radius>RADIUS</radius></sphere>\
                </geometry>\
              </collision>\
              <visual name ='visual'>\
                <geometry>\
                  <sphere><radius>RADIUS</radius></sphere>\
                </geometry>\
                  <material>\
                  <ambient>RED GREEN BLUE 1</ambient>\
                  <diffuse>RED GREEN BLUE 1</diffuse>\
                  </material>\
              </visual>\
              <gravity>GRAVITY</gravity>\
            </link>\
          </model>\
        </sdf>");

  replace(sphereSDF_string, "POSEX", pose.x);
  replace(sphereSDF_string, "POSEY", pose.y);
  replace(sphereSDF_string, "POSEZ", pose.z);
  replace(sphereSDF_string, "RADIUS", radius);
  replace(sphereSDF_string, "GRAVITY", 0);
    
  replace(sphereSDF_string, "ROLL", pose.roll);
  replace(sphereSDF_string, "PITCH", pose.pitch);
  replace(sphereSDF_string, "YAW", pose.yaw);

  replace(sphereSDF_string, "RED", color.r/255.0);
  replace(sphereSDF_string, "GREEN", color.g/255.0);
  replace(sphereSDF_string, "BLUE", color.b/255.0);

  sphereSDF.SetFromString(sphereSDF_string);

  int nobjects=++objects.count;
  ostringstream objlabel;
  objlabel << "sphere"<< nobjects;

  sdf::ElementPtr model = getSDFRoot(sphereSDF)->GetElement("model");

  model->GetAttribute("name")->SetFromString(objlabel.str());

  world->InsertModelSDF(sphereSDF);

  physics::ModelPtr tmp=world->GetModel(objlabel.str());
  objects.insert(pair<string,physics::ModelPtr>(objlabel.str(), tmp));

  if (isSynchronous())
     waitForEngine();

  return objlabel.str();
}

string WorldProxy::makeBox(const double width, const double height, const double thickness, const GazeboYarpPlugins::Pose& pose, const GazeboYarpPlugins::Color& color)
{
  sdf::SDF boxSDF;

  string boxSDF_String=string(
    "<?xml version='1.0'?>\
    <sdf version ='1.4'>\
        <model name ='box'>\
      <pose>POSEX POSEY POSEZ  ROLL PITCH YAW</pose>\
      <link name ='link'>\
      <pose>0 0 0 0 0 0</pose>\
      <collision name ='collision'>\
        <geometry>\
          <box><size>WIDTH HEIGHT THICKNESS</size></box>\
        </geometry>\
      </collision>\
      <visual name='visual'>\
        <geometry>\
          <box><size>WIDTH HEIGHT THICKNESS</size></box>\
        </geometry>\
        <material>\
                   <ambient>RED GREEN BLUE 1</ambient>\
                   <diffuse>RED GREEN BLUE 1</diffuse>\
        </material>\
       </visual>\
       <gravity>GRAVITY</gravity>\
       </link>\
      </model>\
  </sdf>");

  replace(boxSDF_String, "POSEX", pose.x);
  replace(boxSDF_String, "POSEY", pose.y);
  replace(boxSDF_String, "POSEZ", pose.z);

  replace(boxSDF_String, "ROLL", pose.roll);
  replace(boxSDF_String, "PITCH", pose.pitch);
  replace(boxSDF_String, "YAW", pose.yaw);

  replace(boxSDF_String, "WIDTH", width);
  replace(boxSDF_String, "HEIGHT", height);
  replace(boxSDF_String, "THICKNESS", thickness);

  replace(boxSDF_String, "RED", color.r/255.0);
  replace(boxSDF_String, "GREEN", color.g/255.0);
  replace(boxSDF_String, "BLUE", color.b/255.0);
  
  replace(boxSDF_String, "GRAVITY", 0);

  boxSDF.SetFromString(boxSDF_String);

  int nobjects=++objects.count;
  ostringstream objlabel;
  objlabel << "box"<<nobjects;

  sdf::ElementPtr model = getSDFRoot(boxSDF)->GetElement("model");

  model->GetAttribute("name")->SetFromString(objlabel.str());
  world->InsertModelSDF(boxSDF);

  physics::ModelPtr tmp=world->GetModel(objlabel.str());
  objects.insert(std::pair<string,physics::ModelPtr>(objlabel.str(), tmp));

  if (isSynchronous())
     waitForEngine();

  return objlabel.str();
}

string WorldProxy::makeCylinder(const double radius, const double length, const GazeboYarpPlugins::Pose& pose, const GazeboYarpPlugins::Color& color)
{
  sdf::SDF cylSDF;

  string cylSDF_String=string(
    "<?xml version='1.0'?>\
    <sdf version ='1.4'>\
        <model name ='cylinder'>\
      <pose>POSEX POSEY POSEZ  ROLL PITCH YAW</pose>\
         <link name ='link'>\
      <pose>0 0 0 0 0 0</pose>\
      <collision name ='collision'>\
        <geometry>\
          <cylinder><radius>RADIUS</radius><length>LENGTH</length></cylinder>\
        </geometry>\
      </collision>\
      <visual name='visual'>\
        <geometry>\
          <cylinder><radius>RADIUS</radius><length>LENGTH</length></cylinder>\
      </geometry>\
      <material>\
                   <ambient>RED GREEN BLUE 1</ambient>\
                   <diffuse>RED GREEN BLUE 1</diffuse>\
          </material>\
    </visual>\
    <gravity>GRAVITY</gravity>\
    </link>\
      </model>\
  </sdf>");

  replace(cylSDF_String, "POSEX", pose.x);
  replace(cylSDF_String, "POSEY", pose.y);
  replace(cylSDF_String, "POSEZ", pose.z);
  replace(cylSDF_String, "RADIUS", radius);
  replace(cylSDF_String, "LENGTH", length);

  replace(cylSDF_String, "ROLL", pose.roll);
  replace(cylSDF_String, "PITCH", pose.pitch);
  replace(cylSDF_String, "YAW", pose.yaw);

  replace(cylSDF_String, "RED", color.r/255.0);
  replace(cylSDF_String, "GREEN", color.g/255.0);
  replace(cylSDF_String, "BLUE", color.b/255.0);
  
  replace(cylSDF_String, "GRAVITY", 0);

  cylSDF.SetFromString(cylSDF_String);

  int nobjects=++objects.count;
  ostringstream objlabel;
  objlabel << "cylinder"<<nobjects;

  sdf::ElementPtr model = getSDFRoot(cylSDF)->GetElement("model");


  model->GetAttribute("name")->SetFromString(objlabel.str());
  world->InsertModelSDF(cylSDF);

  physics::ModelPtr tmp=world->GetModel(objlabel.str());
  objects.insert(std::pair<string,physics::ModelPtr>(objlabel.str(), tmp));

  if (isSynchronous())
     waitForEngine();

  return objlabel.str();
}

bool WorldProxy::setPose(const std::string& id, const GazeboYarpPlugins::Pose& pose)
{
  physics::ModelPtr model=world->GetModel(id);
    if (!model)
    {
      yError()<<"Object " << id << " does not exist in gazebo\n";
      return false;
    }

  PoseCmd cmd;
  cmd.name=id;


  cmd.pose=math::Pose(pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);
  mutex.lock();
  posecommands.push(cmd);
  mutex.unlock();

  if (isSynchronous())
     waitForEngine();

  return true;
}

bool WorldProxy::enableGravity(const std::string& id, const bool enable)
{
  physics::ModelPtr model=world->GetModel(id);
    if (!model)
    {
      yError() <<"Object " << id << " does not exist in gazebo";
      return false;
    }

  model->SetGravityMode(enable);
  if (enable==true) yInfo("Gravity enabled for model %s", id.c_str());
  else yInfo("Gravity disabled for model %s", id.c_str());
    
  if (isSynchronous())
     waitForEngine();

  return true;
}


std::string WorldProxy::makeFrame(const double size, const GazeboYarpPlugins::Pose& pose, const GazeboYarpPlugins::Color& color)
{
  sdf::SDF frameSDF;
/*
    string frameSDF_string=string(
      "<?xml version='1.0'?>\
       <sdf version ='1.4'>\
          <model name ='frame'>\
            <pose>POSEX POSEY POSEZ ROLL PITCH YAW</pose>\
            \
            <link name ='link_z'>\
              <pose>0 0 HLENGHT 0 0 0</pose>\
              <visual name ='visual'>\
                <geometry>\
                   <cylinder><radius>RADIUS</radius><length>LENGHT</length></cylinder>\
                </geometry>\
                  <material>\
                  <ambient>0 0 1 1</ambient>\
                  <diffuse>0 0 1 1</diffuse>\
                  </material>\
              </visual>\
              <gravity>GRAVITY</gravity>\
            </link>\
            \
            <link name ='link_y'>\
              <pose>0 HLENGHT 0 1.5707 0 0</pose>\
              <visual name ='visual'>\
                <geometry>\
                   <cylinder><radius>RADIUS</radius><length>LENGHT</length></cylinder>\
                </geometry>\
                  <material>\
                  <ambient>0 1 0 1</ambient>\
                  <diffuse>0 1 0 1</diffuse>\
                  </material>\
              </visual>\
              <gravity>GRAVITY</gravity>\
            </link>\
            \
            <link name ='link_x'>\
              <pose>HLENGHT 0 0 0 1.5707 0</pose>\
              <visual name ='visual'>\
                <geometry>\
                   <cylinder><radius>RADIUS</radius><length>LENGHT</length></cylinder>\
                </geometry>\
                  <material>\
                  <ambient>1 0 0 1</ambient>\
                  <diffuse>1 0 0 1</diffuse>\
                  </material>\
              </visual>\
              <gravity>GRAVITY</gravity>\
            </link>\
            <link name ='ball'>\
      <pose>0 0 0 0 0 0</pose>\
      <visual name='visual'>\
        <geometry>\
                  <sphere><radius>BRADIUS</radius></sphere>\
        </geometry>\
        <material>\
                   <ambient>RED GREEN BLUE 1</ambient>\
                   <diffuse>RED GREEN BLUE 1</diffuse>\
        </material>\
       </visual>\
       <gravity>GRAVITY</gravity>\
       </link>\
          </model>\
        </sdf>");
    */
  string frameSDF_string=string(
      "<?xml version='1.0'?>\
       <sdf version ='1.4'>\
          <model name ='frame'>\
            <pose>POSEX POSEY POSEZ ROLL PITCH YAW</pose>\
            \
            <link name ='link'>\
             \
              <visual name ='visual_z'>\
                <pose>0 0 HLENGHT 0 0 0</pose>\
                <geometry>\
                   <cylinder><radius>RADIUS</radius><length>LENGHT</length></cylinder>\
                </geometry>\
                  <material>\
                  <ambient>0 0 1 1</ambient>\
                  <diffuse>0 0 1 1</diffuse>\
                  </material>\
              </visual>\
              \
              <visual name ='visual_y'>\
              <pose>0 HLENGHT 0 1.5707 0 0</pose>\
                <geometry>\
                   <cylinder><radius>RADIUS</radius><length>LENGHT</length></cylinder>\
                </geometry>\
                  <material>\
                  <ambient>0 1 0 1</ambient>\
                  <diffuse>0 1 0 1</diffuse>\
                  </material>\
              </visual>\
              \
              <visual name ='visual_x'>\
              <pose>HLENGHT 0 0 0 1.5707 0</pose>\
                <geometry>\
                   <cylinder><radius>RADIUS</radius><length>LENGHT</length></cylinder>\
                </geometry>\
                  <material>\
                  <ambient>1 0 0 1</ambient>\
                  <diffuse>1 0 0 1</diffuse>\
                  </material>\
              </visual>\
            \
             <visual name='visual_ball'>\
             <pose>0 0 0 0 0 0</pose>\
             <geometry>\
               <sphere><radius>BRADIUS</radius></sphere>\
              </geometry>\
             <material>\
               <ambient>RED GREEN BLUE 1</ambient>\
                <diffuse>RED GREEN BLUE 1</diffuse>\
               </material>\
            </visual>\
            <gravity>0</gravity>\
            <inertial>\
                <mass>1e-21</mass>\
            </inertial>\
            \
            </link>\
          </model>\
        </sdf>");

  replace(frameSDF_string, "POSEX", pose.x);
  replace(frameSDF_string, "POSEY", pose.y);
  replace(frameSDF_string, "POSEZ", pose.z);
  replace(frameSDF_string, "HLENGHT", size/2);
  replace(frameSDF_string, "LENGHT", size);
  replace(frameSDF_string, "BRADIUS", 0.06);
  replace(frameSDF_string, "RADIUS", 0.04);
    
  replace(frameSDF_string, "ROLL", pose.roll);
  replace(frameSDF_string, "PITCH", pose.pitch);
  replace(frameSDF_string, "YAW", pose.yaw);

  replace(frameSDF_string, "RED", color.r/255.0);
  replace(frameSDF_string, "GREEN", color.g/255.0);
  replace(frameSDF_string, "BLUE", color.b/255.0);

  frameSDF.SetFromString(frameSDF_string);

  int nobjects=++objects.count;
  ostringstream objlabel;
  objlabel << "frame"<< nobjects;

  sdf::ElementPtr model = getSDFRoot(frameSDF)->GetElement("model");

  model->GetAttribute("name")->SetFromString(objlabel.str());

  world->InsertModelSDF(frameSDF);

  physics::ModelPtr tmp=world->GetModel(objlabel.str());
  objects.insert(pair<string,physics::ModelPtr>(objlabel.str(), tmp));

  if (isSynchronous())
     waitForEngine();

  return objlabel.str();
}

bool WorldProxy::changeColor(const std::string& id, const GazeboYarpPlugins::Color& color)
{
  physics::ModelPtr model=world->GetModel(id);
    if (!model)
    {
      yError() <<"Object " << id << " does not exist in gazebo";
      return false;
    }
    
  if (isSynchronous())
     waitForEngine();

  return true;
}
  
bool WorldProxy::enableCollision(const std::string& id, const bool enable)
{
  physics::ModelPtr model=world->GetModel(id);
    if (!model)
    {
      yError() <<"Object " << id << " does not exist in gazebo";
      return false;
    }

  if (enable==true) {model->SetCollideMode("all");yInfo("Collision detection enabled for model %s", id.c_str());}
  else {model->SetCollideMode("none");yInfo("Collision detection disabled for model %s", id.c_str());}
    
  if (isSynchronous())
     waitForEngine();

  return true;
}

gazebo::physics::LinkPtr WorldProxy::HELPER_getLinkInModel(gazebo::physics::ModelPtr model, std::string link_name)
{
    gazebo::physics::Link_V model_links = model->GetLinks();
    for(int i=0; i < model_links.size(); i++ )
    {
        std::string candidate_link = model_links[i]->GetScopedName();
        if( HELPER_hasEnding(candidate_link,"::"+link_name) )
	{
            return model_links[i];
        }
    }
    return gazebo::physics::LinkPtr();
}

bool WorldProxy::HELPER_hasEnding (std::string const &fullString, std::string const &ending)
{
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

bool WorldProxy::attach(const std::string& id, const std::string& link_name)
{
    physics::ModelPtr object_model_1=world->GetModel(id);
    if (!object_model_1)
    {
      yError() <<"Object " << id << " does not exist in gazebo";
      return false;
    }
    
    physics::ModelPtr object_model_2=world->GetModel("SIM_CER_ROBOT");
    if (!object_model_2)
    {
      yError() <<"Object " << "SIM_CER_ROBOT" << " does not exist in gazebo";
      return false;
    }
    
    physics::JointPtr joint;
    joint = world->GetPhysicsEngine()->CreateJoint("revolute", object_model_1);
    if( !joint )
    {
        yError() << "Unable to create joint";
        return false;
    }

    physics::LinkPtr parent_link = object_model_1->GetLink();
    physics::LinkPtr object_link = HELPER_getLinkInModel(object_model_2,link_name);

    if( !object_link )
    {
        yError() << "Unable to get object_link";
        return false;
    }
    if( !parent_link )
    {
        yError() << "Unable to get parent link";
        return false;
    }
    
    //math::Pose parent_link_pose = parent_link->GetWorldCoGPose();
    //object_link->SetWorldPose(parent_link_pose);

    //TODO add mutex
    joint->SetName("magnet_joint");
    joint->SetModel(object_model_1);
    joint->Load(parent_link, object_link, gazebo::math::Pose());
    joint->Attach(parent_link, object_link);
    joint->SetHighStop(0, 0);
    joint->SetLowStop(0, 0);
     object_model_1->LoadJoints();
     object_model_2->LoadJoints();
    //joint->SetParam("cfm", 0, 0);
        yDebug() << object_model_1->GetJointCount() << object_model_2->GetJointCount();

    return true;
}

bool WorldProxy::detach(const std::string& id)
{
    physics::ModelPtr object_model=world->GetModel(id);
    if (!object_model)
    {
      yError() <<"Object " << id << " does not exist in gazebo";
      return false;
    }
    
    yError() << "^^" << object_model->GetJointCount();
    physics::JointPtr joint = object_model->GetJoint ("magnet_joint");
    if (!joint)
    {
      yError() <<"Joint not found";
      return false;
    }
    
    return true;
}


GazeboYarpPlugins::Pose WorldProxy::getPose(const std::string& id)
{
  GazeboYarpPlugins::Pose ret;

  physics::ModelPtr model=world->GetModel(id);
    if (!model)
    {
      yError()<<"Object " << id << " does not exist in gazebo\n";
      return ret;
    }


  math::Pose p=model->GetWorldPose();

  ret.x=p.pos[0];
  ret.y=p.pos[1];
  ret.z=p.pos[2];
  ret.roll=p.rot.GetRoll();
  ret.pitch=p.rot.GetPitch();
  ret.yaw=p.rot.GetYaw();

  return ret;
}

bool WorldProxy::deleteObject(const std::string& id)
{
  ObjectsListIt it=objects.begin();
  while(it!=objects.end())
  {
    string obj=it->first;
    physics::ModelPtr model=world->GetModel(obj);
    if (model)
    {
      world->RemoveModel(obj);
      objects.erase(it);
      return true;
    }
    else
    {
      it++;
    }
  }
  return false;
}

bool WorldProxy::deleteAll()
{
  ObjectsListIt it=objects.begin();
  while(it!=objects.end())
  {
    world->RemoveModel(it->first);
    it++;
  }

  objects.clear();

  // RemoveModel is blocking no need to synchronize
  // if (isSynchronous())
  //   waitForEngine();

  return true;
}

std::vector<std::string> WorldProxy::getList()
{
  ObjectsListIt it=objects.begin();

  //first update the list because objects can be removed from the gui too...
  while(it!=objects.end())
  {
    string obj=it->first;
    physics::ModelPtr model=world->GetModel(obj);
    if (!model)
    {
      it=objects.erase(it);
    }
    else
    {
      it++;
    }
  }

  //...then fill the return vector
  vector<std::string> ret;
  it=objects.begin();
  while(it!=objects.end())
  {
     ret.push_back(it->first);
     it++;  
  }
  
  return ret;
}

bool WorldProxy::loadModelFromFile(const std::string& filename)
{
  yError()<<"loadFromModelFile not yet implemented\n";
  return false;
}

void WorldProxy::update(const common::UpdateInfo & _info)
{
  mutex.lock();

  while(!posecommands.empty())
  {
    PoseCmd cmd=posecommands.front();

    physics::ModelPtr model=world->GetModel(cmd.name);
    if (!model)
    {
      yError()<<"Object " << cmd.name << " does not exist in gazebo\n";
    }

    model->SetWorldPose(cmd.pose);

    posecommands.pop();
  }

  mutex.unlock();

  synchHelper.signalAll();
}

void WorldProxy::waitForEngine()
{
  synchHelper.wait();
}