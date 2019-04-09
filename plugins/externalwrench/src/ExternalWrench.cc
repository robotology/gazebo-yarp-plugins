/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "ExternalWrench.hh"
#include <random>

int ExternalWrench::count = 0;

//Initializing wrench command
ExternalWrench::ExternalWrench()
{
    // Increase wrench count
    count++;

    // Initialize visual color
    std::mt19937 rand_gen(50); //fixed seed
    std::uniform_real_distribution<> uni_dis(0.0, 1.0);

    for(int i = 0; i < count; i++)
    {
        uni_dis(rand_gen);
        if (i == count-1) {
            color[0] = uni_dis(rand_gen);
            color[1] = uni_dis(rand_gen);
            color[2] = uni_dis(rand_gen);
            color[3] = uni_dis(rand_gen);
        }
    }

    // Default wrench values
    wrenchPtr->link_name = "world";
    wrenchPtr->force.resize(3,0);
    wrenchPtr->torque.resize(3,0);
    wrenchPtr->duration = 0.0;

    tick = yarp::os::Time::now();
    duration_done = false;
}

bool ExternalWrench::getLink()
{
    // Get the exact link with only link name instead of full_scoped_link_name
    physics::Link_V links = this->model->GetLinks();
    for(int i=0; i < links.size(); i++)
    {
        std::string candidate_link_name = links[i]->GetScopedName();

        // hasEnding compare ending of the condidate model link name to the given link name, in order to be able to use unscoped names
        if(GazeboYarpPlugins::hasEnding(candidate_link_name, wrenchPtr->link_name))
        {
            link = links[i];
            break;
        }
    }

    if(!link)
    {
        return false;
    }

    return true;
}

void ExternalWrench::setVisual()
{
    //Wrench Visual
    node = transport::NodePtr(new gazebo::transport::Node());
    this->node->Init(model->GetWorld()->Name());
    visPub = this->node->Advertise<msgs::Visual>("~/visual",100);

    // Set the visual's name. This should be unique.
    std::string visual_name = "GYP_EXT_WRENCH__" + wrenchPtr->link_name + "__CYLINDER_VISUAL__" + std::to_string(count);
    visualMsg.set_name (visual_name);

    // Set the visual's parent. This visual will be attached to the parent
    visualMsg.set_parent_name(model->GetScopedName());

    // Create a cylinder
    msgs::Geometry *geomMsg = visualMsg.mutable_geometry();
    geomMsg->set_type(msgs::Geometry::CYLINDER);
    geomMsg->mutable_cylinder()->set_radius(0.0075);
    geomMsg->mutable_cylinder()->set_length(.30);

    // Don't cast shadows
    visualMsg.set_cast_shadows(false);
}

bool ExternalWrench::setWrench(physics::ModelPtr& _model,yarp::os::Bottle& cmd)
{
    model = _model;

    //get link name from command
    wrenchPtr->link_name = cmd.get(0).asString();

    if(getLink())
    {
        setVisual();

        wrenchPtr->force[0]  =  cmd.get(1).asDouble();
        wrenchPtr->force[1]  =  cmd.get(2).asDouble();
        wrenchPtr->force[2]  =  cmd.get(3).asDouble();

        wrenchPtr->torque[0] = cmd.get(4).asDouble();
        wrenchPtr->torque[1] = cmd.get(5).asDouble();
        wrenchPtr->torque[2] = cmd.get(6).asDouble();

        wrenchPtr->duration  = cmd.get(7).asDouble();

        return true;
    }
    else return false;
}

void ExternalWrench::applyWrench()
{
    tock = yarp::os::Time::now();
    if((tock-tick) < wrenchPtr->duration)
    {
#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Vector3d force (wrenchPtr->force[0], wrenchPtr->force[1], wrenchPtr->force[2]);
        ignition::math::Vector3d torque (wrenchPtr->torque[0], wrenchPtr->torque[1], wrenchPtr->torque[2]);

        link->AddForce(force);
        link->AddTorque(torque);

        ignition::math::Vector3d linkCoGPos = link->WorldCoGPose().Pos(); // Get link's COG position where wrench will be applied
        ignition::math::Vector3d newZ = force.Normalize(); // normalized force. I want the z axis of the cylinder's reference frame to coincide with my force vector
        ignition::math::Vector3d newX = newZ.Cross (ignition::math::Vector3d::UnitZ);
        ignition::math::Vector3d newY = newZ.Cross (newX);
        ignition::math::Matrix4d rotation = ignition::math::Matrix4d (newX[0],newY[0],newZ[0],0,newX[1],newY[1],newZ[1],0,newX[2],newY[2],newZ[2],0, 0, 0, 0, 1);
        ignition::math::Quaterniond forceOrientation = rotation.Rotation();
        ignition::math::Pose3d linkCoGPose (linkCoGPos - rotation*ignition::math::Vector3d ( 0,0,.15 ), forceOrientation);
#else
        math::Vector3d force (wrenchPtr->force[0], wrenchPtr->force[1], wrenchPtr->force[2]);
        math::Vector3d torque (wrenchPtr->torque[0], wrenchPtr->torque[1], wrenchPtr->torque[2]);

        link->AddForce(force);
        link->AddTorque(torque);

        math::Vector3d linkCoGPos = link->WorldCoGPose().Pos(); // Get link's COG position where wrench will be applied
        math::Vector3d newZ = force.Normalize(); // normalized force. I want the z axis of the cylinder's reference frame to coincide with my force vector
        math::Vector3d newX = newZ.Cross (ignition::math::Vector3d::UnitZ);
        math::Vector3d newY = newZ.Cross (newX);
        math::Matrix4d rotation = ignition::math::Matrix4d (newX[0],newY[0],newZ[0],0,newX[1],newY[1],newZ[1],0,newX[2],newY[2],newZ[2],0, 0, 0, 0, 1);
        math::Quaterniond forceOrientation = rotation.Rotation();
        math::Pose3d linkCoGPose (linkCoGPos - rotation*ignition::math::Vector3d ( 0,0,.15 ), forceOrientation);
#endif

#if GAZEBO_MAJOR_VERSION == 7
        msgs::Set(visualMsg.mutable_pose(), linkCoGPose.Ign());
#else
        msgs::Set(visualMsg.mutable_pose(), linkCoGPose);
#endif
#if GAZEBO_MAJOR_VERSION >= 9
        msgs::Set(visualMsg.mutable_material()->mutable_ambient(), ignition::math::Color(color[0],color[1],color[2],color[3]));
#else
        msgs::Set(visualMsg.mutable_material()->mutable_ambient(),common::Color(color[0],color[1],color[2],color[3]));
#endif
        visualMsg.set_visible(1);
        visPub->Publish(visualMsg);
    }
    else
    {
        deleteWrench();
    }
}

void ExternalWrench::deleteWrench()
{
    this->wrenchPtr->link_name.clear();
    this->wrenchPtr->force.clear();
    this->wrenchPtr->torque.clear();
    this->wrenchPtr->duration = 0;

    this->visualMsg.set_visible(0);
    this->visualMsg.clear_geometry();
    this->visualMsg.clear_delete_me();
    this->visPub->Publish(visualMsg);
    this->duration_done = true;
}

ExternalWrench::~ExternalWrench()
{
    count--;
}
