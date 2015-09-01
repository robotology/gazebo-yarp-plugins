// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2015 iCub Facility 
 * Authors: Lorenzo Natale
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
 
 #include "worldproxy.h"

#include <math.h>

#include <string>
#include <iostream>
#include <ostream>

using namespace std;
using namespace gazebo;

#include <yarp/os/Time.h>

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

std::string WorldProxy::makeSphere(const double radius, const GazeboYarpPlugins::Pose& pose, const GazeboYarpPlugins::Color& color)
{
  sdf::SDF sphereSDF;
 
  string sphereSDF_string=string(
       "<sdf version ='1.4'>\
          <model name ='sphere'>\
            <pose>POSEX POSEY POSEZ ROLL PITCH YAW</pose>\
            <link name ='link'>\
              <pose>POSEX POSEY POSEZ ROLL PITCH YAW</pose>\
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

  replace(sphereSDF_string, "POSEX", pose.x);
  replace(sphereSDF_string, "POSEY", pose.y);
  replace(sphereSDF_string, "POSEZ", pose.z);
  replace(sphereSDF_string, "RADIUS", radius);
  
  replace(sphereSDF_string, "ROLL", pose.roll);
  replace(sphereSDF_string, "PITCH", pose.pitch);
  replace(sphereSDF_string, "YAW", pose.yaw);
  
  sphereSDF.SetFromString(sphereSDF_string);
  
  int nobjects=++objects.count;
  ostringstream objlabel;
  objlabel << "sphere"<< nobjects;
  
  sdf::ElementPtr model = sphereSDF.root->GetElement("model");

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
      <pose>POSEX POSEY POSEZ ROLL PITCH YAW</pose>\
      <collision name ='collision'>\
        <geometry>\
          <box><size>WIDTH HEIGHT THICKNESS</size></box>\
        </geometry>\
      </collision>\
      <visual name='visual'>\
        <geometry>\
          <box><size>WIDTH HEIGHT THICKNESS</size></box>\
	  </geometry>\
	</visual>\
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
  
  
  boxSDF.SetFromString(boxSDF_String);
  
  int nobjects=++objects.count;
  ostringstream objlabel;
  objlabel << "box"<<nobjects;
  
  sdf::ElementPtr model = boxSDF.root->GetElement("model");
  
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
      <pose>POSEX POSEY POSEZ ROLL PITCH YAW</pose>\
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

  replace(cylSDF_String, "POSEX", pose.x);
  replace(cylSDF_String, "POSEY", pose.y);
  replace(cylSDF_String, "POSEZ", pose.z);
  replace(cylSDF_String, "RADIUS", radius);
  replace(cylSDF_String, "LENGTH", length);
  
  replace(cylSDF_String, "ROLL", pose.roll);
  replace(cylSDF_String, "PITCH", pose.pitch);
  replace(cylSDF_String, "YAW", pose.yaw);
    
  
  cylSDF.SetFromString(cylSDF_String);
  
  int nobjects=++objects.count;
  ostringstream objlabel;
  objlabel << "cylinder"<<nobjects;
  
  sdf::ElementPtr model = cylSDF.root->GetElement("model");
  
 
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
      cerr<<"Object " << id << " does not exist in gazebo\n";
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


GazeboYarpPlugins::Pose WorldProxy::getPose(const std::string& id)
{
  GazeboYarpPlugins::Pose ret;
    
  physics::ModelPtr model=world->GetModel(id);
    if (!model)
    {
      cerr<<"Object " << id << " does not exist in gazebo\n";
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
  vector<std::string> ret;
  
  ObjectsListIt it=objects.begin();
  while(it!=objects.end())
  {
    ret.push_back(it->first);
    it++;
  }
  
  return ret;
}

bool WorldProxy::loadModelFromFile(const std::string& filename)
{
  cerr<<"loadFromModelFile not yet implemented\n";
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
      cerr<<"Object " << cmd.name << " does not exist in gazebo\n";
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