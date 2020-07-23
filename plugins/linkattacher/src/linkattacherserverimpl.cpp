/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include <linkattacherserverimpl.h>

#include <algorithm>
#include <vector>

using namespace gazebo;
using namespace std;
using namespace yarp::os;

LinkAttacherServerImpl::LinkAttacherServerImpl()
{}

LinkAttacherServerImpl::~LinkAttacherServerImpl()
{}

bool LinkAttacherServerImpl::attachUnscoped(const string& parent_model_name, const string& parent_model_link_name, const string& child_model_name, const string& child_model_link_name)
{
    #if GAZEBO_MAJOR_VERSION >= 8
    gazebo::physics::ModelPtr parent_model = _world->ModelByName(parent_model_name);
    #else
    gazebo::physics::ModelPtr parent_model = _world->GetModel(parent_model_name);
    #endif

    if(!parent_model)
    {
        yError() << LogPrefix << "Attach: " << parent_model_name << " model does not exist in gazebo";
        return false;
    }
    else
    {
      yInfo() << LogPrefix << "Attach: " << parent_model->GetName() << " model found";
    }

    //Get the exact link with only link name instead of full_scoped_link_name
    gazebo::physics::Link_V parent_model_links = parent_model->GetLinks();
    gazebo::physics::LinkPtr parent_model_link;
    for(int i=0; i < parent_model_links.size(); i++)
    {
        std::string candidate_parent_model_link_name = parent_model_links[i]->GetScopedName();
        
        // hasEnding compare ending of the condidate model link name to the given link name, in order to be able to use unscoped names
        if(GazeboYarpPlugins::hasEnding(candidate_parent_model_link_name, parent_model_link_name))
        {
            parent_model_link = parent_model_links[i];
            yInfo() << LogPrefix << "Attach: " << parent_model_link->GetName() << " link found";
            break;
        }
    }
    if(!parent_model_link)
    {
    	yError() << LogPrefix << "Attach: " << parent_model_link_name << " link is not found";
    	return false;
    }

    #if GAZEBO_MAJOR_VERSION >= 8
    gazebo::physics::ModelPtr child_model = _world->ModelByName(child_model_name);
    #else
    gazebo::physics::ModelPtr child_model = _world->GetModel(child_model_name);
    #endif

    if(!child_model)
    {
        yError() << LogPrefix << "Attach: " << child_model_name << " model does not exist in gazebo";
        return false;
    }
    else
    {
      yInfo() << LogPrefix << "Attach: " << child_model->GetName() << " model found";
    }

    //Get the exact link with only link name instead of full_scoped_link_name
    gazebo::physics::Link_V child_model_links = child_model->GetLinks();
    gazebo::physics::LinkPtr child_model_link;
    for(int i=0; i < child_model_links.size(); i++)
    {
        std::string candidate_child_model_link_name = child_model_links[i]->GetScopedName();
        // hasEnding compare ending of the condidate model link name to the given link name, in order to be able to use unscoped names
        if(GazeboYarpPlugins::hasEnding(candidate_child_model_link_name, child_model_link_name))
        {
            child_model_link = child_model_links[i];
            yInfo() << LogPrefix << "Attach: " << child_model_link->GetName() << " link found";
            break;
        }
    }
    if(!child_model_link)
    {
    	yError() << LogPrefix << "Attach: " << child_model_link_name << " link is not found";
    	return false;
    }

    //This is joint creation
    gazebo::physics::JointPtr joint;

    #if GAZEBO_MAJOR_VERSION >= 8
    joint = _world->Physics()->CreateJoint(jointType,parent_model);
    #else
    joint = _world->GetPhysicsEngine()->CreateJoint(jointType,parent_model);
    #endif

    if(!joint)
    {
        yError() << LogPrefix << "Attach: Unable to create joint";
        return false;
    }

    std::string joint_name = parent_model_link_name + "_" + jointType + "_joint";
    joint->SetName(joint_name);
    yInfo() << LogPrefix << "Attach: joint: " << joint->GetName() << " created";

    joint->SetModel(parent_model);

    #if GAZEBO_MAJOR_VERSION >= 8
    joint->Load(parent_model_link,child_model_link,ignition::math::Pose3d());
    #else
    joint->Load(parent_model_link,child_model_link,gazebo::math::Pose());
    #endif

    //Attach(parent_link,child_link)
    joint->Attach(parent_model_link,child_model_link);

    return true;
}

bool LinkAttacherServerImpl::detachUnscoped(const string& model_name, const string& model_link_name)
{
    #if GAZEBO_MAJOR_VERSION >= 8
    gazebo::physics::ModelPtr model = _world->ModelByName(model_name);
    #else
    gazebo::physics::ModelPtr model = _world->GetModel(model_name);
    #endif

    if(!model)
    {
        yError() << LogPrefix << "Detach: " << model_name << " model does not exist in gazebo";
        return false;
    }
    else
    {
      yInfo() << LogPrefix << "Detach: " << model->GetName() << " model found";
    }

    //Get the exact link with only link name instead of full_scoped_link_name
    gazebo::physics::Link_V model_links = model->GetLinks();
    gazebo::physics::LinkPtr model_link;
    for(int i=0; i < model_links.size(); i++)
    {
        std::string candidate_model_link_name = model_links[i]->GetScopedName();
        // hasEnding compare ending of the condidate model link name to the given link name, in order to be able to use unscoped names
        if(GazeboYarpPlugins::hasEnding(candidate_model_link_name, model_link_name))
        {
            model_link = model_links[i];
            yInfo() << LogPrefix << "Detach: " << model_link->GetName() << " link found";
            break;
        }
    }
    if(!model_link)
    {
    	yError() << LogPrefix << "Detach: " << model_link_name << " link is not found";
    	return false;
    }

    std::string joint_name = model_name + "::" + model_link_name + "_" + jointType + "_joint";

    //Get all the joints at the object link
    gazebo::physics::Joint_V joints_v = model_link->GetChildJoints();
    gazebo::physics::JointPtr joint;
    for(int i=0; i < joints_v.size(); i++)
    {
        std::string candidate_joint_name = joints_v[i]->GetScopedName();
        if(candidate_joint_name == joint_name)
        {
            joint = joints_v[i];
            if(joint)
            {
                joint->Detach();
                yInfo() << LogPrefix << "Detach: Found joint: " << joint->GetName() << " detached";
            }
        }
    }
    if(!joint)
    {
        yError() << LogPrefix << "Detach: Joint not found";
        return false;
    }
    return true;
}

bool LinkAttacherServerImpl::enableGravity(const string& model_name, const bool enable)
{
  #if GAZEBO_MAJOR_VERSION >= 8
  gazebo::physics::ModelPtr model = _world->ModelByName(model_name);
  #else
  gazebo::physics::ModelPtr model = _world->GetModel(model_name);
  #endif

  if(!model)
  {
    yError() << LogPrefix << "enableGravity: " << model_name << " does not exist in gazebo";
    return false;
  }

  model->SetGravityMode(enable);
  if(enable==true)
  {
    yInfo() << LogPrefix << "enableGravity: Gravity enabled for model " << model_name.c_str();
  }
  else
  {
    yInfo() << LogPrefix << "enableGravity: Gravity disabled for model " << model_name.c_str();
  }

  return true;
}

bool LinkAttacherServerImpl::setJointType(const std::string j)
{
    if(std::find(jointTypes.begin(), jointTypes.end(), j) != jointTypes.end())
    {
        jointType=j;
    }
    else
    {
        yError() << LogPrefix << "the choosen joint type [" << j << "] is not valid";
        return false;
    }

    return true;
}
