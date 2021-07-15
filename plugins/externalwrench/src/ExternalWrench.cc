/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "ExternalWrench.hh"
#include <random>

// Initializing wrench command
ExternalWrench::ExternalWrench()
{
    // Default wrench index
    wrenchIndex = 0;

    // Default color
    color[0] = 0;
    color[1] = 0;
    color[2] = 0;
    color[3] = 0;

    // Default wrench values
    wrench.link_name = "world";
    wrench.force.resize(3,0);
    wrench.torque.resize(3,0);
    wrench.duration = 0.0;

    tick = 0; // Default tick value
    tock = 0; // Default tock value
    duration_done = false;

    wrenchSmoothingFlag = false;
    timeStepIndex = 0;
}

void ExternalWrench::setWrenchColor()
{
    // Set visual color
    std::mt19937 rand_gen(this->wrenchIndex); //fixed seed based on wrench index
    std::uniform_real_distribution<> uni_dis(0.0, 1.0);

    color[0] = uni_dis(rand_gen);
    color[1] = uni_dis(rand_gen);
    color[2] = uni_dis(rand_gen);
    color[3] = uni_dis(rand_gen);
}

void ExternalWrench::setWrenchIndex(int& index)
{
    this->wrenchIndex = index;
}

void ExternalWrench::setTick(double& tickTime)
{
    this->tick = tickTime;
}

void ExternalWrench::setTock(double& tockTime)
{
    this->tock = tockTime;
}

bool ExternalWrench::getLink()
{
    // Get the exact link with only link name instead of full_scoped_link_name
    physics::Link_V links = this->model->GetLinks();
    for(int i=0; i < links.size(); i++)
    {
        std::string candidate_link_name = links[i]->GetScopedName();

        // hasEnding compare ending of the condidate model link name to the given link name, in order to be able to use unscoped names
        if(GazeboYarpPlugins::hasEnding(candidate_link_name, wrench.link_name))
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
    // Wrench Visual
    node = transport::NodePtr(new gazebo::transport::Node());
    this->node->Init(model->GetWorld()->Name());
    visPub = this->node->Advertise<msgs::Visual>("~/visual",100);

    // Set the visual's name. This should be unique.
    std::string visual_name = "GYP_EXT_WRENCH__" + wrench.link_name + "__CYLINDER_VISUAL__" + std::to_string(wrenchIndex);
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

bool ExternalWrench::setWrench(physics::ModelPtr& _model,yarp::os::Bottle& cmd, const double& simulationUpdatePeriod, const bool& wrenchSmoothing)
{
    model = _model;

    // Get link name from command
    wrench.link_name = cmd.get(0).asString();

    if(getLink())
    {
        // Get wrench duration
        wrench.duration  = cmd.get(7).asFloat64();

        if (wrenchSmoothing) {
            wrenchSmoothingFlag = true;
            bool wrenchSmoothingStatus = smoothWrench(cmd, simulationUpdatePeriod);

            if (wrenchSmoothingStatus) {

                // Get the first elements of the smoothed wrench
                yarp::sig::Vector initSmoothedWrench = wrench.smoothedWrenchVec.at(timeStepIndex);

                wrench.force[0] = initSmoothedWrench[0];
                wrench.force[1] = initSmoothedWrench[1];
                wrench.force[2] = initSmoothedWrench[2];

                wrench.torque[0] = initSmoothedWrench[3];
                wrench.torque[1] = initSmoothedWrench[4];
                wrench.torque[2] = initSmoothedWrench[5];

                return true;
            }
            else return  false;
        }
        else {

            wrench.force[0]  =  cmd.get(1).asFloat64();
            wrench.force[1]  =  cmd.get(2).asFloat64();
            wrench.force[2]  =  cmd.get(3).asFloat64();

            wrench.torque[0] = cmd.get(4).asFloat64();
            wrench.torque[1] = cmd.get(5).asFloat64();
            wrench.torque[2] = cmd.get(6).asFloat64();

            return true;
        }

    }
    else return false;
}

bool ExternalWrench::smoothWrench(const yarp::os::Bottle& cmd, const double& simulationUpdatePeriod)
{
    // Clear smoothed wrenches vector
    wrench.smoothedWrenchVec.clear();

    double duration = cmd.get(7).asFloat64();

    // Compute time steps
    steps = duration/simulationUpdatePeriod;

    // Get original wrench
    yarp::sig::Vector originalWrench;
    originalWrench.resize(6,0);

    originalWrench[0]  =  cmd.get(1).asFloat64();
    originalWrench[1]  =  cmd.get(2).asFloat64();
    originalWrench[2]  =  cmd.get(3).asFloat64();
    originalWrench[3]  =  cmd.get(4).asFloat64();
    originalWrench[4]  =  cmd.get(5).asFloat64();
    originalWrench[5]  =  cmd.get(6).asFloat64();

    double time = 0;
    std::vector<double> smoothingCoefficients;

    for (int timeStep = 0; timeStep <= steps; timeStep++) {

        // Second derivative of minimum jerk trajectory equation
        // Reference : https://mika-s.github.io/python/control-theory/trajectory-generation/2017/12/06/trajectory-generation-with-a-minimum-jerk-trajectory.html
        double smoothingCoefficient = duration*(30 * std::pow(time/duration, 2) - 60 * std::pow(time/duration, 3) + 30 * std::pow(time/duration, 4));
        smoothingCoefficients.push_back(smoothingCoefficient);

        // Update time
        time = time + simulationUpdatePeriod;
    }

    // Normalize smoothing coefficients
    double smoothingCoefficientMax = *std::max_element(smoothingCoefficients.begin(), smoothingCoefficients.end());
    for (int timeStep = 0; timeStep <= steps; timeStep++) {
        smoothingCoefficients.at(timeStep) =  smoothingCoefficients.at(timeStep) / smoothingCoefficientMax;
    }

    yarp::sig::Vector smoothedWrench;
    smoothedWrench.resize(6,0);

    for (int timeStep = 0; timeStep <= steps; timeStep++) {

        smoothedWrench = originalWrench * smoothingCoefficients.at(timeStep);

        wrench.smoothedWrenchVec.push_back(smoothedWrench);
        smoothedWrench.clear();
    }

    return true;
}

void ExternalWrench::applyWrench()
{
    if((tock-tick) < wrench.duration)
    {
#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Vector3d force;
        ignition::math::Vector3d torque;

        if (wrenchSmoothingFlag) {

            // Update time step index
            if (timeStepIndex < steps) {
                timeStepIndex++;
            }

            yarp::sig::Vector smoothedWrench = wrench.smoothedWrenchVec.at(timeStepIndex);

            wrench.force[0] = smoothedWrench[0];
            wrench.force[1] = smoothedWrench[1];
            wrench.force[2] = smoothedWrench[2];

            wrench.torque[0] = smoothedWrench[3];
            wrench.torque[1] = smoothedWrench[4];
            wrench.torque[2] = smoothedWrench[5];
        }

        force[0] = wrench.force[0];
        force[1] = wrench.force[1];
        force[2] = wrench.force[2];

        torque[0] = wrench.torque[0];
        torque[1] = wrench.torque[1];
        torque[2] = wrench.torque[2];

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
        math::Vector3d force (wrench.force[0], wrench.force[1], wrench.force[2]);
        math::Vector3d torque (wrench.torque[0], wrench.torque[1], wrench.torque[2]);

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
    this->wrench.link_name.clear();
    this->wrench.force.clear();
    this->wrench.torque.clear();
    this->wrench.duration = 0;

    if (wrenchSmoothingFlag && !this->wrench.smoothedWrenchVec.empty()) {
        this->wrench.smoothedWrenchVec.clear();
    }

    this->visualMsg.set_visible(0);
    this->visualMsg.clear_geometry();
    this->visualMsg.clear_delete_me();
    this->visPub->Publish(visualMsg);
    this->duration_done = true;
}

ExternalWrench::~ExternalWrench()
{}
