/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include <linkattacherserverimpl.h>

using namespace gazebo;
using namespace std;
using namespace yarp::os;

LinkAttacherServerImpl::LinkAttacherServerImpl()
{}

LinkAttacherServerImpl::~LinkAttacherServerImpl()
{}

bool LinkAttacherServerImpl::attachUnscoped(const string& model_name, const std::string& model_link_name, const std::string& robot_name, const string& robot_link_name)
{

    #if GAZEBO_MAJOR_VERSION >= 8
    gazebo::physics::ModelPtr object_model = _world->ModelByName(model_name);
    #else
    gazebo::physics::ModelPtr object_model = _world->GetModel(model_name);
    #endif

    if(!object_model)
    {
        yError() << LogPrefix << "Attach: " << model_name << " does not exist in gazebo";
        return false;
    }
    else
    {
      yInfo() << LogPrefix << "Attach: " << object_model->GetName() << " found";
    }

    gazebo::physics::LinkPtr object_link = object_model->GetLink(model_link_name);
    if(!object_link)
    {
        yError() << LogPrefix << "Attach: " << model_link_name << " is not found";
        return false;
    }
    else
    {
      yInfo() << LogPrefix << "Attach: " << object_link->GetName() << " found";
    }

    #if GAZEBO_MAJOR_VERSION >= 8
    gazebo::physics::ModelPtr robot_model = _world->ModelByName(robot_name);
    #else
    gazebo::physics::ModelPtr robot_model = _world->GetModel(robot_name);
    #endif

    if(!robot_model)
    {
        yError() << LogPrefix << "Attach: " << robot_name << " does not exist in gazebo";
        return false;
    }
    else
    {
      yInfo() << LogPrefix << "Attach: " << robot_model->GetName() << " found";
    }

    //Get the exact link with only link name instead of full_scoped_link_name
    gazebo::physics::Link_V robot_model_links = robot_model->GetLinks();
    for(int i=0; i < robot_model_links.size(); i++)
    {
        //E.g iCub::l_hand::l_hand_base_link

        std::string candidate_robot_link_name = robot_model_links[i]->GetScopedName();

        std::size_t lastcolon = candidate_robot_link_name.rfind(":");
        std::string unscoped_robot_link_name = candidate_robot_link_name.substr(lastcolon+1,std::string::npos);

        if(unscoped_robot_link_name == robot_link_name)
        {
            yInfo() << LogPrefix << "Attach: Full scoped Candidate robot link name: " << candidate_robot_link_name << " found";
            gazebo::physics::LinkPtr robot_link = robot_model_links[i];
            if(!robot_link)
            {
                yError() << LogPrefix << "Attach: " << robot_link_name << " is not found";
                return false;
            }
            else
            {
              yInfo() << LogPrefix << "Attach: " << robot_link->GetName() << " found";
            }

            //This is joint creation
            gazebo::physics::JointPtr joint;

            #if GAZEBO_MAJOR_VERSION >= 8
            joint = _world->Physics()->CreateJoint("fixed",object_model);
            #else
            joint = _world->GetPhysicsEngine()->CreateJoint("fixed",object_model);
            #endif

            if(!joint)
            {
                yError() << LogPrefix << "Attach: Unable to create joint";
                return false;
            }

            std::string joint_name = model_link_name + "_magnet_joint";
            joint->SetName(joint_name);
            yInfo() << LogPrefix << "Attach: Magnet joint: " << joint->GetName() << " created";

            joint->SetModel(object_model);

            #if GAZEBO_MAJOR_VERSION >= 8
            joint->Load(object_link,robot_link,ignition::math::Pose3d());
            #else
            joint->Load(object_link,robot_link,gazebo::math::Pose());
            #endif

            //Attach(prent_link,child_link)
            joint->Attach(object_link,robot_link);

            break;
        }
    }

    return true;
}

bool LinkAttacherServerImpl::detachUnscoped(const string& model_name, const std::string& model_link_name)
{
    #if GAZEBO_MAJOR_VERSION >= 8
    gazebo::physics::ModelPtr object_model = _world->ModelByName(model_name);
    #else
    gazebo::physics::ModelPtr object_model = _world->GetModel(model_name);
    #endif

    if(!object_model)
    {
        yError() << LogPrefix << "Detach: " << model_name << " does not exist in gazebo";
        return false;
    }
    else
    {
      yInfo() << LogPrefix << "Detach: " << object_model->GetName() << " found";
    }

    gazebo::physics::LinkPtr object_link = object_model->GetLink(model_link_name);
    if(!object_link)
    {
        yError() << LogPrefix << "Detach: " << model_link_name << " is not found";
        return false;
    }
    else
    {
      yInfo() << LogPrefix << "Detach: " << object_link->GetName() << " found";
    }

    std::string joint_name = model_name + "::" + model_link_name + "_magnet_joint";

    //Get all the joints at the object link
    gazebo::physics::Joint_V joints_v = object_link->GetChildJoints();

    for(int i=0; i < joints_v.size(); i++)
    {
        std::string candidate_joint_name = joints_v[i]->GetScopedName();
        if(candidate_joint_name == joint_name)
        {
            gazebo::physics::JointPtr joint = joints_v[i];

            if(!joint)
            {
                yError() << LogPrefix << "Detach: Magnet Joint not found";
                return false;
            }
            else
            {
                joint->Detach();
                yInfo() << LogPrefix << "Detach: Found joint: " << joint->GetName() << " detached";
            }
        }
    }
    return true;

}

bool LinkAttacherServerImpl::enableGravity(const std::string& model_name, const bool enable)
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
