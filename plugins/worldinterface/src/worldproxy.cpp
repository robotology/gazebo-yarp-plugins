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

typedef ignition::math::Pose3d YarpWorldPose;

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

std::string WorldProxy::makeSphere(const double radius, const GazeboYarpPlugins::Pose& pose, const GazeboYarpPlugins::Color& color, const std::string& frame_name, const std::string& object_name, const bool gravity_enable, const bool collision_enable)
{
  double timeout = 2.0;
    if (object_name!= "")
    {
        physics::ModelPtr model=world->ModelByName(object_name);
        if (model)
        {
            yError()<<"An object called " << object_name << "exists already in gazebo\n";
            return "";
        }
    }

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

  YarpWorldPose relative_link_pose;
  if (frame_name!="")
  {
    physics::LinkPtr relative_link = HELPER_getLink(frame_name);
    if( !relative_link )
    {
      yError() << "Unable to find specified link";
      return "";
    }
    relative_link_pose = relative_link->WorldPose();
  }
  YarpWorldPose final_pose (pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw) ;
  final_pose += relative_link_pose;

  replace(sphereSDF_string, "POSEX", final_pose.Pos()[0]);
  replace(sphereSDF_string, "POSEY", final_pose.Pos()[1]);
  replace(sphereSDF_string, "POSEZ", final_pose.Pos()[2]);
  replace(sphereSDF_string, "RADIUS", radius);
  if (gravity_enable) {replace (sphereSDF_string, "GRAVITY", 1);}
  else {replace (sphereSDF_string, "GRAVITY", 0);}

  replace(sphereSDF_string, "ROLL", final_pose.Rot().Roll());
  replace(sphereSDF_string, "PITCH", final_pose.Rot().Pitch());
  replace(sphereSDF_string, "YAW", final_pose.Rot().Yaw());

  replace(sphereSDF_string, "RED", color.r/255.0);
  replace(sphereSDF_string, "GREEN", color.g/255.0);
  replace(sphereSDF_string, "BLUE", color.b/255.0);

  sphereSDF.SetFromString(sphereSDF_string);

  int nobjects=++objects.count;
  ostringstream objlabel;

  if (object_name != "")
  {
      objlabel << object_name;
  }
  else
  {
     objlabel << "sphere"<< nobjects;
  }
  sdf::ElementPtr model = getSDFRoot(sphereSDF)->GetElement("model");

  model->GetAttribute("name")->SetFromString(objlabel.str());
  world->InsertModelSDF(sphereSDF);

    physics::ModelPtr tmp=world->ModelByName(objlabel.str());
  bool insert = (tmp == NULL);

  // the insert is non blocking; need to finish the insertion before request it
  double start = yarp::os::SystemClock::nowSystem();
  double time = yarp::os::SystemClock::nowSystem() - start;

  while (insert || (time<timeout))
  {
    tmp=world->ModelByName(objlabel.str());
    insert = (tmp == NULL);
    yarp::os::SystemClock::delaySystem(0.1);
    time = yarp::os::SystemClock::nowSystem() - start;
  }

  if (tmp == 0)
  {
    yWarning() << "Internal error during object creation. Unimplemented feature in gazebo 7.";
    return "";
  }
  else
  {
    if (collision_enable) {tmp->SetCollideMode("all");}
    else {tmp->SetCollideMode("none");}
  }
  objects.insert(pair<string,physics::ModelPtr>(objlabel.str(), tmp));

  if (isSynchronous())
     waitForEngine();

  return objlabel.str();
}

string WorldProxy::makeBox(const double width, const double height, const double thickness, const GazeboYarpPlugins::Pose& pose, const GazeboYarpPlugins::Color& color, const std::string& frame_name, const std::string& object_name, const bool gravity_enable, const bool collision_enable)
{
  double timeout = 2.0;
    if (object_name!= "")
    {
        physics::ModelPtr model=world->ModelByName(object_name);
        if (model)
        {
            yError()<<"An object called " << object_name << "exists already in gazebo\n";
            return "";
        }
    }
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

  YarpWorldPose relative_link_pose;
  if (frame_name!="")
  {
    physics::LinkPtr relative_link = HELPER_getLink(frame_name);
    if( !relative_link )
    {
      yError() << "Unable to find specified link";
      return "";
    }
    relative_link_pose = relative_link->WorldPose();
  }
  YarpWorldPose final_pose (pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw) ;
  final_pose += relative_link_pose;

  replace(boxSDF_String, "POSEX", final_pose.Pos()[0]);
  replace(boxSDF_String, "POSEY", final_pose.Pos()[1]);
  replace(boxSDF_String, "POSEZ", final_pose.Pos()[2]);
  replace(boxSDF_String, "ROLL", final_pose.Rot().Roll());
  replace(boxSDF_String, "PITCH", final_pose.Rot().Pitch());
  replace(boxSDF_String, "YAW", final_pose.Rot().Yaw());

  replace(boxSDF_String, "WIDTH", width);
  replace(boxSDF_String, "HEIGHT", height);
  replace(boxSDF_String, "THICKNESS", thickness);

  replace(boxSDF_String, "RED", color.r/255.0);
  replace(boxSDF_String, "GREEN", color.g/255.0);
  replace(boxSDF_String, "BLUE", color.b/255.0);

  if (gravity_enable) {replace (boxSDF_String, "GRAVITY", 1);}
  else {replace (boxSDF_String, "GRAVITY", 0);}

  boxSDF.SetFromString(boxSDF_String);

  int nobjects=++objects.count;
  ostringstream objlabel;
  if (object_name!= "")
  {
     objlabel << object_name;
  }
  else
  {
    objlabel << "box" << nobjects;
  }

  sdf::ElementPtr model = getSDFRoot(boxSDF)->GetElement("model");

  model->GetAttribute("name")->SetFromString(objlabel.str());
  world->InsertModelSDF(boxSDF);

  physics::ModelPtr tmp=world->ModelByName(objlabel.str());

  bool insert = (tmp == NULL);

  // the insert is non blocking; need to finish the insertion before request it
  double start = yarp::os::SystemClock::nowSystem();
  double time = yarp::os::SystemClock::nowSystem() - start;

  while (insert || (time<timeout))
  {
    tmp=world->ModelByName(objlabel.str());
    insert = (tmp == NULL);
    yarp::os::SystemClock::delaySystem(0.1);
    time = yarp::os::SystemClock::nowSystem() - start;
  }

  if (tmp == 0)
  {
    yWarning() << "Internal error during object creation. Unimplemented feature in gazebo 7.";
    return "";
  }
  else
  {
    if (collision_enable) {tmp->SetCollideMode("all");}
    else {tmp->SetCollideMode("none");}
  }
  objects.insert(pair<string,physics::ModelPtr>(objlabel.str(), tmp));

  if (isSynchronous())
     waitForEngine();

  return objlabel.str();
}

string WorldProxy::makeCylinder(const double radius, const double length, const GazeboYarpPlugins::Pose& pose, const GazeboYarpPlugins::Color& color, const std::string& frame_name, const std::string& object_name, const bool gravity_enable, const bool collision_enable)
{
  double timeout = 2.0;
    if (object_name!= "")
    {
        physics::ModelPtr model=world->ModelByName(object_name);
        if (model)
        {
            yError()<<"An object called " << object_name << "exists already in gazebo\n";
            return "";
        }
    }

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

  YarpWorldPose relative_link_pose;
  if (frame_name!="")
  {
    physics::LinkPtr relative_link = HELPER_getLink(frame_name);
    if( !relative_link )
    {
      yError() << "Unable to find specified link";
      return "";
    }
    relative_link_pose = relative_link->WorldPose();
  }
  YarpWorldPose final_pose (pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw) ;
  final_pose += relative_link_pose;

  replace(cylSDF_String, "POSEX", final_pose.Pos()[0]);
  replace(cylSDF_String, "POSEY", final_pose.Pos()[1]);
  replace(cylSDF_String, "POSEZ", final_pose.Pos()[2]);
  replace(cylSDF_String, "ROLL", final_pose.Rot().Roll());
  replace(cylSDF_String, "PITCH", final_pose.Rot().Pitch());
  replace(cylSDF_String, "YAW", final_pose.Rot().Yaw());

  replace(cylSDF_String, "RADIUS", radius);
  replace(cylSDF_String, "LENGTH", length);

  replace(cylSDF_String, "RED", color.r/255.0);
  replace(cylSDF_String, "GREEN", color.g/255.0);
  replace(cylSDF_String, "BLUE", color.b/255.0);

  if (gravity_enable) {replace (cylSDF_String, "GRAVITY", 1);}
  else {replace (cylSDF_String, "GRAVITY", 0);}

  cylSDF.SetFromString(cylSDF_String);

  int nobjects=++objects.count;
  ostringstream objlabel;
  if (object_name!= "")
  {
     objlabel << object_name;
  }
  else
  {
    objlabel << "cylinder" << nobjects;
  }

  sdf::ElementPtr model = getSDFRoot(cylSDF)->GetElement("model");


  model->GetAttribute("name")->SetFromString(objlabel.str());
  world->InsertModelSDF(cylSDF);

  physics::ModelPtr tmp=world->ModelByName(objlabel.str());
  bool insert = (tmp == NULL);

  // the insert is non blocking; need to finish the insertion before request it
  double start = yarp::os::SystemClock::nowSystem();
  double time = yarp::os::SystemClock::nowSystem() - start;

  while (insert || (time<timeout))
  {
    tmp=world->ModelByName(objlabel.str());
    insert = (tmp == NULL);
    yarp::os::SystemClock::delaySystem(0.1);
    time = yarp::os::SystemClock::nowSystem() - start;
  }

  if (tmp == 0)
  {
    yWarning() << "Internal error during object creation. Unimplemented feature in gazebo 7.";
    return "";
  }
  else
  {
    if (collision_enable) {tmp->SetCollideMode("all");}
    else {tmp->SetCollideMode("none");}
  }
  objects.insert(pair<string,physics::ModelPtr>(objlabel.str(), tmp));

  if (isSynchronous())
     waitForEngine();

  return objlabel.str();
}

bool WorldProxy::setPose(const std::string& id, const GazeboYarpPlugins::Pose& pose, const std::string& frame_name)
{
    physics::ModelPtr model=world->ModelByName(id);
    if (!model)
    {
      yError()<<"Object " << id << " does not exist in gazebo\n";
      return false;
    }

  PoseCmd cmd;
  cmd.name=id;

  YarpWorldPose relative_link_pose;
  if (frame_name!="")
  {
    physics::LinkPtr relative_link = HELPER_getLink(frame_name);
    if( !relative_link )
    {
      yError() << "Unable to find specified link";
      return false;
    }
    relative_link_pose = relative_link->WorldPose();
  }
  YarpWorldPose final_pose (pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw) ;
  final_pose += relative_link_pose;

  cmd.pose=final_pose;
  mutex.lock();
  posecommands.push(cmd);
  mutex.unlock();

  if (isSynchronous())
     waitForEngine();

  return true;
}

bool WorldProxy::enableGravity(const std::string& id, const bool enable)
{
    physics::ModelPtr model=world->ModelByName(id);
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


std::string WorldProxy::makeFrame(const double size, const GazeboYarpPlugins::Pose& pose, const GazeboYarpPlugins::Color& color, const std::string& frame_name, const std::string& object_name, const bool gravity_enable, const bool collision_enable)
{
  double timeout = 2.0;
    if (object_name!= "")
    {
        physics::ModelPtr model=world->ModelByName(object_name);
        if (model)
        {
            yError()<<"An object called " << object_name << "exists already in gazebo\n";
            return "";
        }
    }

    sdf::SDF frameSDF;

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
            <gravity>GRAVITY</gravity>\
            <inertial>\
                <mass>1e-21</mass>\
            </inertial>\
            \
            </link>\
          </model>\
        </sdf>");

  YarpWorldPose relative_link_pose;
  if (frame_name!="")
  {
    physics::LinkPtr relative_link = HELPER_getLink(frame_name);
    if( !relative_link )
    {
      yError() << "Unable to find specified link";
      return "";
    }
    relative_link_pose = relative_link->WorldPose();
  }
  YarpWorldPose final_pose (pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw) ;
  final_pose += relative_link_pose;

  replace(frameSDF_string, "POSEX", final_pose.Pos()[0]);
  replace(frameSDF_string, "POSEY", final_pose.Pos()[1]);
  replace(frameSDF_string, "POSEZ", final_pose.Pos()[2]);
  replace(frameSDF_string, "ROLL", final_pose.Rot().Roll());
  replace(frameSDF_string, "PITCH", final_pose.Rot().Pitch());
  replace(frameSDF_string, "YAW", final_pose.Rot().Yaw());

  replace(frameSDF_string, "HLENGHT", size/2);
  replace(frameSDF_string, "LENGHT", size);
  replace(frameSDF_string, "BRADIUS", 0.06);
  replace(frameSDF_string, "RADIUS", 0.04);

  replace(frameSDF_string, "RED", color.r/255.0);
  replace(frameSDF_string, "GREEN", color.g/255.0);
  replace(frameSDF_string, "BLUE", color.b/255.0);
  if (gravity_enable) {replace (frameSDF_string, "GRAVITY", 1);}
  else {replace (frameSDF_string, "GRAVITY", 0);}

  frameSDF.SetFromString(frameSDF_string);

  int nobjects=++objects.count;
  ostringstream objlabel;
  if (object_name!= "")
  {
     objlabel << object_name;
  }
  else
  {
    objlabel << "frame" << nobjects;
  }

  sdf::ElementPtr model = getSDFRoot(frameSDF)->GetElement("model");

  model->GetAttribute("name")->SetFromString(objlabel.str());
  world->InsertModelSDF(frameSDF);

    physics::ModelPtr tmp=world->ModelByName(objlabel.str());

  bool insert = (tmp == NULL);

  // the insert is non blocking; need to finish the insertion before request it
  double start = yarp::os::SystemClock::nowSystem();
  double time = yarp::os::SystemClock::nowSystem() - start;

  while (insert || (time<timeout))
  {
    tmp=world->ModelByName(objlabel.str());
    insert = (tmp == NULL);
    yarp::os::SystemClock::delaySystem(0.1);
    time = yarp::os::SystemClock::nowSystem() - start;
  }

  if (tmp == 0)
  {
    yWarning() << "Internal error during object creation. Unimplemented feature in gazebo 7.";
    return "";
  }
  else
  {
    if (collision_enable) {tmp->SetCollideMode("all");}
    else {tmp->SetCollideMode("none");}
  }
  objects.insert(pair<string,physics::ModelPtr>(objlabel.str(), tmp));

  if (isSynchronous())
     waitForEngine();

  return objlabel.str();
}

bool WorldProxy::changeColor(const std::string& id, const GazeboYarpPlugins::Color& color)
{
    physics::ModelPtr model=world->ModelByName(id);
    if (!model)
    {
      yError() <<"Object " << id << " does not exist in gazebo";
      return false;
    }


  // TO BE COMPLETED
  // msgs::Visual visualMsg;
  // visualMsg.set_name(_name);
  // visualMsg.set_parent_name(_parentName);
  // visualMsg.set_transparency(0);
  // this->visualPub->Publish(visualMsg);

  if (isSynchronous())
     waitForEngine();

  return true;
}

bool WorldProxy::enableCollision(const std::string& id, const bool enable)
{
    physics::ModelPtr model=world->ModelByName(id);
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

gazebo::physics::LinkPtr WorldProxy::HELPER_getLink(std::string full_scoped_link_name)
{
    size_t firstcolon = full_scoped_link_name.find(":");
    if (firstcolon == std::string::npos)
    {
      yError () << "Unable to parse model name: " << full_scoped_link_name;
      return gazebo::physics::LinkPtr();
    }
    std::string model_name = full_scoped_link_name.substr(0,firstcolon);
    physics::ModelPtr p_model=world->ModelByName(model_name);
    if (!p_model)
    {
      yError () << "Unable to find model: " << model_name;
      return gazebo::physics::LinkPtr();
    }

    gazebo::physics::Link_V model_links = p_model->GetLinks();
    for(int i=0; i < model_links.size(); i++ )
    {
        std::string candidate_link = model_links[i]->GetScopedName();
        if( candidate_link==full_scoped_link_name )
	{
            return model_links[i];
        }
    }
    yError () << "Unable to find link: " << full_scoped_link_name << "belonging to model: " <<model_name;
    return gazebo::physics::LinkPtr();
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

bool WorldProxy::rename(const std::string& old_name, const std::string& new_name)
{
    physics::ModelPtr object_model_1=world->ModelByName(old_name);
    if (!object_model_1)
    {
      yError() <<"Object " << old_name << " does not exist in gazebo";
      return false;
    }
    physics::ModelPtr object_model_2=world->ModelByName(new_name);
    if (object_model_2)
    {
      yError() <<"Object " << new_name << " already exists in gazebo";
      return false;
    }

    ObjectsListIt old_it=objects.begin();
    bool found_old = false;
    ObjectsListIt new_it=objects.begin();
    bool found_new = false;

    while(old_it!=objects.end())
    {
       string obj=old_it->first;
       if (obj==old_name)
       {
         found_old = true;
         break;
       }
       old_it++;
    }
    if (!found_old)
    {
       yError() <<"Object " << old_name << " not found in objects list";
       return false;
    }
    while(new_it!=objects.end())
    {
       string obj=new_it->first;
       if (obj==new_name)
       {
         found_new= true;
         break;
       }
       new_it++;
    }
    if (found_new)
    {
       yError() <<"Object " << new_name << " already exists in objects list";
       return false;
    }

    yWarning()<<"model->setName() not fully implemented by gazebo";
    object_model_1->SetName(new_name);
    objects.erase(old_it);
    objects.insert(pair<string,physics::ModelPtr>(new_name, object_model_1));

    yInfo() << "Object "<< old_name << " renamed to: " << new_name;
    return true;
}

bool WorldProxy::attach(const std::string& id, const std::string& link_name)
{
    physics::ModelPtr object_model_1=world->ModelByName(id);
    if (!object_model_1)
    {
      yError() <<"Object " << id << " does not exist in gazebo";
      return false;
    }

    /*
    size_t lastcolon = link_name.rfind(":");
    if (lastcolon == std::string::npos)
    {
      yError () << "Unable to parse model name: " << link_name;
      return false;
    }
    std::string model_name = link_name.substr(0,lastcolon-1);
    physics::ModelPtr object_model_2=world->GetModel(model_name);
    if (!object_model_2)
    {
      yError() <<"Object " << model_name << " does not exist in gazebo";
      return false;
    }*/

    physics::JointPtr joint;
    physics::PhysicsEnginePtr physics=world->Physics();
    joint = physics->CreateJoint("revolute", object_model_1);
    if( !joint )
    {
        yError() << "Unable to create joint";
        return false;
    }

    physics::LinkPtr parent_link = object_model_1->GetLink();
    physics::LinkPtr object_link = HELPER_getLink(link_name);
    //physics::LinkPtr object_link = HELPER_getLinkInModel(object_model_2,link_name);

    if( !object_link )
    {
        yError() << "Unable to get object_link: " << link_name;
        return false;
    }
    if( !parent_link )
    {
        yError() << "Unable to get parent link: " << id;
        return false;
    }

    //YarpWorldPose parent_link_pose = parent_link->GetWorldPose();
    //object_link->SetWorldPose(parent_link_pose);

    //TODO add mutex
    joint->SetName("magnet_joint");
    joint->SetModel(object_model_1);
    joint->Load(parent_link, object_link, YarpWorldPose());
    joint->Attach(parent_link, object_link);
    joint->SetUpperLimit(0, 0);
    joint->SetLowerLimit(0, 0);
    //joint->SetParam("cfm", 0, 0);
    //yDebug() << object_model_1->GetJointCount() << object_model_2->GetJointCount();

    return true;
}

bool WorldProxy::detach(const std::string& id)
{
    physics::ModelPtr object_model=world->ModelByName(id);
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


GazeboYarpPlugins::Pose WorldProxy::getPose(const std::string& id, const std::string& frame_name)
{
  GazeboYarpPlugins::Pose ret;

  physics::ModelPtr model=world->ModelByName(id);
    if (!model)
    {
      yError()<<"Object " << id << " does not exist in gazebo\n";
      return ret;
    }

    YarpWorldPose relative_link_pose;
    if (frame_name!="")
    {
        physics::LinkPtr relative_link = HELPER_getLink(frame_name);
        if( !relative_link )
        {
            yError() << "Unable to find specified link";
            return ret;
        }
        relative_link_pose = relative_link->WorldPose();
    }

    YarpWorldPose p=model->WorldPose();
    YarpWorldPose final_pose (p.Pos()[0], p.Pos()[1], p.Pos()[2], p.Rot().Roll(), p.Rot().Pitch(),p.Rot().Yaw());


    final_pose -= relative_link_pose;

    ret.x=final_pose.Pos()[0];
    ret.y=final_pose.Pos()[1];
    ret.z=final_pose.Pos()[2];
    ret.roll=final_pose.Rot().Roll();
    ret.pitch=final_pose.Rot().Pitch();
    ret.yaw=final_pose.Rot().Yaw();


  return ret;
}

bool WorldProxy::deleteObject(const std::string& id)
{
    physics::ModelPtr model=world->ModelByName(id);
    if (model)
    {
        world->RemoveModel(id);
        objects.erase(id);
        return true;
    }
    else
    {
        yError()<<"WorldProxy::deleteObject: the obj called" << id << "not exists";
        return false;
    }
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
    physics::ModelPtr model=world->ModelByName(obj);
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
  double timeout = 2.0;
  sdf::SDFPtr modelSDF_ptr(new sdf::SDF());
  if (!sdf::init(modelSDF_ptr))
  {
    yError() << "SDF init failed";
    return false;
  }

  if (!sdf::readFile(filename, modelSDF_ptr))
  {
    yError() << "SDF parsing the xml failed";
    return false;
  }

  GazeboYarpPlugins::Pose pose;
  std::vector<double> pose_tmp;
  stringstream pose_stream;

  sdf::ElementPtr model = modelSDF_ptr->Root()->GetElement("model");
  pose_stream << *model->GetElement("pose")->GetValue();
  double x;
  while(pose_stream >> x)
    pose_tmp.push_back(x);

  if (pose_tmp.size() == 6)
  {
    pose.x = pose_tmp[0];
    pose.y = pose_tmp[1];
    pose.z = pose_tmp[2];
    pose.roll = pose_tmp[3];
    pose.pitch = pose_tmp[4];
    pose.yaw = pose_tmp[5];
    return !loadModelFromFileWithPose(filename, pose, "", timeout).empty();
  }
  else
  {
    yError() << "Pose does not contain 6 values!";
    return false;
  }
}

std::string WorldProxy::loadModelFromFileWithPose(const std::string &filename, const GazeboYarpPlugins::Pose& pose, const std::string& object_name, const double timeout)
{
  if (object_name != "")
  {
      physics::ModelPtr model=world->ModelByName(object_name);
      if (model)
      {
          yError() << "An object called " << object_name << "exists already in gazebo\n";
          return "";
      }
  }

  string model_name = "";

  sdf::SDFPtr modelSDF_ptr(new sdf::SDF());
  if (!sdf::init(modelSDF_ptr))
  {
    yError() << "SDF init failed";
    return "";
  }

  std::string abspath = sdf::findFile(filename);
  if (!abspath.empty())
  {
    if (!sdf::readFile(abspath, modelSDF_ptr))
    {
      yError() << "SDF parsing the xml failed";
      return "";
    }
  }
  else
  {
    yError() << "File not found! An absolute path is needed!";
    return "";
  }

  sdf::ElementPtr model = modelSDF_ptr->Root()->GetElement("model");
  ostringstream objlabel;
  int nobjects = ++objects.count;

  if (object_name.empty())
    model_name = model->GetAttribute("name")->GetAsString();
  else
    model_name = object_name;

    physics::ModelPtr test=world->ModelByName(model_name);

  objlabel << model_name;
  if (test)
  {
    objlabel << nobjects;
  }

  ostringstream model_pose;
  model_pose << pose.x << " " << pose.y << " " <<  pose.z << " " <<  pose.roll << " " <<  pose.pitch << " " <<  pose.yaw;

  model->GetAttribute("name")->SetFromString(objlabel.str());
  model->GetElement("pose")->Set(pose.toString());

  world->InsertModelSDF(*modelSDF_ptr);
    physics::ModelPtr tmp=world->ModelByName(objlabel.str());

  // the insert is non blocking; need to finish the insertion before request it
  double start = yarp::os::SystemClock::nowSystem();
  double time = yarp::os::SystemClock::nowSystem() - start;

  while (!tmp || (time<timeout))
  {
    tmp=world->ModelByName(objlabel.str());
    yarp::os::SystemClock::delaySystem(0.1);
    time = yarp::os::SystemClock::nowSystem() - start;
  }

  if (!tmp)
  {
    yError() << "Internal error during object creation. Unimplemented feature in gazebo 7.";
    return "";
  }
  objects.insert(pair<string, physics::ModelPtr>(objlabel.str(), tmp));

  if (isSynchronous())
    waitForEngine();

  return objlabel.str();
}

void WorldProxy::update(const common::UpdateInfo & _info)
{
  mutex.lock();

  while(!posecommands.empty())
  {
    PoseCmd cmd=posecommands.front();
    physics::ModelPtr model=world->ModelByName(cmd.name);
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
