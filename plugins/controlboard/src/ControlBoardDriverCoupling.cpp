/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: Marco Randazzo <marco.randazzo@iit.it>
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "ControlBoardDriver.h"
#include <GazeboYarpPlugins/common.h>

#include <ControlBoardDriverCoupling.h>
#include <array>
#include <cstdio>
#include <cmath>
#include <gazebo/physics/Model.hh>

#include "ControlBoardLog.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using GazeboYarpPlugins::GAZEBOCONTROLBOARD;

//------------------------------------------------------------------------------------------------------------------
// BaseCouplingHandler
//------------------------------------------------------------------------------------------------------------------

BaseCouplingHandler::BaseCouplingHandler(gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names, std::vector<Range> coupled_joint_limits)
{
    m_robot = model;
    m_coupledJoints=coupled_joints;
    m_coupledJointNames=coupled_joint_names;

    // Configure a map between coupled joints and limits
    for (std::size_t i = 0, j = 0; i < coupled_joints.size(); i++)
    {
        const int coupled_joint_index = coupled_joints(i);
        const std::string coupled_joint_name = getCoupledJointName(coupled_joint_index);
        if (coupled_joint_name != "gyp_invalid" && coupled_joint_name != "reserved")
        {
            m_coupledJointLimits[coupled_joints[i]] = coupled_joint_limits[j];
            j++;
        }
    }
    //m_couplingSize = m_coupledJoints.size();
}

BaseCouplingHandler::~BaseCouplingHandler() {}

bool BaseCouplingHandler::checkJointIsCoupled(int joint)
{
    for (size_t i = 0; i < m_coupledJoints.size(); ++i)
    {
        if (m_coupledJoints[i]==joint) return true;
    }
    return false;
}

yarp::sig::VectorOf<int> BaseCouplingHandler::getCoupledJoints()
{
    return m_coupledJoints;
}

std::string BaseCouplingHandler::getCoupledJointName(int joint)
{
    int c_joint = -1;
    for (size_t i = 0; i < m_coupledJoints.size(); ++i)
    {
        if (m_coupledJoints[i]==joint) c_joint = i;
    }

    if (c_joint >= 0 && static_cast<size_t>(c_joint) < m_coupledJoints.size())
    {
        return m_coupledJointNames[c_joint];
    }
    else
    {
        return std::string("gyp_invalid");
    }
}

void BaseCouplingHandler::setCoupledJointLimit(int joint, const double& min, const double& max)
{
    const std::string coupled_joint_name = getCoupledJointName(joint);

    if (coupled_joint_name != "reserved" && coupled_joint_name != "gyp_invalid")
    {
        m_coupledJointLimits.at(joint).min = min;
        m_coupledJointLimits.at(joint).max = max;
    }
}

void BaseCouplingHandler::getCoupledJointLimit(int joint, double& min, double& max)
{
    const std::string coupled_joint_name = getCoupledJointName(joint);

    if (coupled_joint_name != "reserved" && coupled_joint_name != "gyp_invalid")
    {
        min = m_coupledJointLimits.at(joint).min;
        max = m_coupledJointLimits.at(joint).max;
    }
}

//------------------------------------------------------------------------------------------------------------------
// EyesCouplingHandler
//------------------------------------------------------------------------------------------------------------------

EyesCouplingHandler::EyesCouplingHandler(gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names, std::vector<Range> coupled_joint_limits)
: BaseCouplingHandler(model, coupled_joints,coupled_joint_names, coupled_joint_limits)
{
    m_couplingSize = 2;
}

bool EyesCouplingHandler::decouplePos (yarp::sig::Vector& current_pos)
{
    if (m_coupledJoints.size() != m_couplingSize) return false;
    double temp = current_pos[m_coupledJoints[0]];
    current_pos[m_coupledJoints[0]] = temp + current_pos[m_coupledJoints[1]];
    current_pos[m_coupledJoints[1]] = temp - current_pos[m_coupledJoints[1]];
    return true;
}

bool EyesCouplingHandler::decoupleVel (yarp::sig::Vector& current_vel)
{
    if (m_coupledJoints.size() != m_couplingSize) return false;
    double temp = current_vel[m_coupledJoints[0]];
    current_vel[m_coupledJoints[0]] = temp + current_vel[m_coupledJoints[1]];
    current_vel[m_coupledJoints[1]] = temp - current_vel[m_coupledJoints[1]];
    return true;
}

bool EyesCouplingHandler::decoupleAcc (yarp::sig::Vector& current_acc)
{
    if (m_coupledJoints.size() != m_couplingSize) return false;
    double temp = current_acc[m_coupledJoints[0]];
    current_acc[m_coupledJoints[0]] = temp + current_acc[m_coupledJoints[1]];
    current_acc[m_coupledJoints[1]] = temp - current_acc[m_coupledJoints[1]];
    return true;
}

bool EyesCouplingHandler::decoupleTrq (yarp::sig::Vector& current_trq)
{
    if (m_coupledJoints.size() != m_couplingSize) return false;
    return false;
}

yarp::sig::Vector EyesCouplingHandler::decoupleRefPos (yarp::sig::Vector& pos_ref)
{
    yarp::sig::Vector out = pos_ref;
    if (m_coupledJoints.size() != m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "EyesCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = (pos_ref[m_coupledJoints[0]] + pos_ref[m_coupledJoints[1]])/2;
    out[m_coupledJoints[1]] = (pos_ref[m_coupledJoints[0]] - pos_ref[m_coupledJoints[1]])/2;
    return out;
}

yarp::sig::Vector EyesCouplingHandler::decoupleRefVel (yarp::sig::Vector& vel_ref, const yarp::sig::Vector& pos_feedback)
{
    yarp::sig::Vector out = vel_ref;
    if (m_coupledJoints.size() != m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "EyesCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = (vel_ref[m_coupledJoints[0]] + vel_ref[m_coupledJoints[1]])/2;
    out[m_coupledJoints[1]] = (vel_ref[m_coupledJoints[0]] - vel_ref[m_coupledJoints[1]])/2;
    return out;
}

yarp::sig::Vector EyesCouplingHandler::decoupleRefTrq (yarp::sig::Vector& trq_ref)
{
    yarp::sig::Vector out = trq_ref;
    if (m_coupledJoints.size() != m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "EyesCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = (trq_ref[m_coupledJoints[0]] + trq_ref[m_coupledJoints[1]])/2;
    out[m_coupledJoints[1]] = (trq_ref[m_coupledJoints[0]] - trq_ref[m_coupledJoints[1]])/2;
    return out;
}

//------------------------------------------------------------------------------------------------------------------
// ThumbCouplingHandler
//------------------------------------------------------------------------------------------------------------------

ThumbCouplingHandler::ThumbCouplingHandler(gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names, std::vector<Range> coupled_joint_limits)
: BaseCouplingHandler(model, coupled_joints,coupled_joint_names, coupled_joint_limits)
{
    m_couplingSize = 4;
}

bool ThumbCouplingHandler::decouplePos (yarp::sig::Vector& current_pos)
{
    if (m_coupledJoints.size() != m_couplingSize) return false;
    current_pos[m_coupledJoints[2]] = current_pos[m_coupledJoints[2]] + current_pos[m_coupledJoints[3]];
    return true;
}

bool ThumbCouplingHandler::decoupleVel (yarp::sig::Vector& current_vel)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    current_vel[m_coupledJoints[2]] = current_vel[m_coupledJoints[2]] + current_vel[m_coupledJoints[3]];
    return true;
}

bool ThumbCouplingHandler::decoupleAcc (yarp::sig::Vector& current_acc)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    current_acc[m_coupledJoints[2]] = current_acc[m_coupledJoints[2]] + current_acc[m_coupledJoints[3]];
    return true;
}

bool ThumbCouplingHandler::decoupleTrq (yarp::sig::Vector& current_trq)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    return false;
}

yarp::sig::Vector ThumbCouplingHandler::decoupleRefPos (yarp::sig::Vector& pos_ref)
{
    yarp::sig::Vector out = pos_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "ThumbCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = pos_ref[m_coupledJoints[0]];
    out[m_coupledJoints[1]] = pos_ref[m_coupledJoints[1]];
    out[m_coupledJoints[2]] = pos_ref[m_coupledJoints[2]]/2;
    out[m_coupledJoints[3]] = pos_ref[m_coupledJoints[2]]/2;
    return out;
}

yarp::sig::Vector ThumbCouplingHandler::decoupleRefVel (yarp::sig::Vector& vel_ref, const yarp::sig::Vector& pos_feedback)
{
    yarp::sig::Vector out = vel_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "ThumbCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = vel_ref[m_coupledJoints[0]];
    out[m_coupledJoints[1]] = vel_ref[m_coupledJoints[1]];
    out[m_coupledJoints[2]] = vel_ref[m_coupledJoints[2]]/2;
    out[m_coupledJoints[3]] = vel_ref[m_coupledJoints[2]]/2;
    return out;
}

yarp::sig::Vector ThumbCouplingHandler::decoupleRefTrq (yarp::sig::Vector& trq_ref)
{
    yarp::sig::Vector out =trq_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "ThumbCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = trq_ref[m_coupledJoints[0]];
    out[m_coupledJoints[1]] = trq_ref[m_coupledJoints[1]];
    out[m_coupledJoints[2]] = trq_ref[m_coupledJoints[2]]/2;
    out[m_coupledJoints[3]] = trq_ref[m_coupledJoints[2]]/2;
    return out;
}

//------------------------------------------------------------------------------------------------------------------
// IndexCouplingHandler
//------------------------------------------------------------------------------------------------------------------

IndexCouplingHandler::IndexCouplingHandler(gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names, std::vector<Range> coupled_joint_limits)
: BaseCouplingHandler(model, coupled_joints,coupled_joint_names, coupled_joint_limits)
{
    m_couplingSize = 3;
}

bool IndexCouplingHandler::decouplePos (yarp::sig::Vector& current_pos)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    current_pos[m_coupledJoints[1]] = current_pos[m_coupledJoints[1]] + current_pos[m_coupledJoints[2]];
    return true;
}

bool IndexCouplingHandler::decoupleVel (yarp::sig::Vector& current_vel)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    current_vel[m_coupledJoints[1]] = current_vel[m_coupledJoints[1]] + current_vel[m_coupledJoints[2]];
    return true;
}

bool IndexCouplingHandler::decoupleAcc (yarp::sig::Vector& current_acc)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    current_acc[m_coupledJoints[1]] = current_acc[m_coupledJoints[1]] + current_acc[m_coupledJoints[2]];
    return true;
}

bool IndexCouplingHandler::decoupleTrq (yarp::sig::Vector& current_trq)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    return false;
}

yarp::sig::Vector IndexCouplingHandler::decoupleRefPos (yarp::sig::Vector& pos_ref)
{
    yarp::sig::Vector out = pos_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "IndexCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = pos_ref[m_coupledJoints[0]];
    out[m_coupledJoints[1]] = pos_ref[m_coupledJoints[1]]/2;
    out[m_coupledJoints[2]] = pos_ref[m_coupledJoints[1]]/2;
    return out;
}

yarp::sig::Vector IndexCouplingHandler::decoupleRefVel (yarp::sig::Vector& vel_ref, const yarp::sig::Vector& pos_feedback)
{
    yarp::sig::Vector out = vel_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "IndexCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = vel_ref[m_coupledJoints[0]];
    out[m_coupledJoints[1]] = vel_ref[m_coupledJoints[1]]/2;
    out[m_coupledJoints[2]] = vel_ref[m_coupledJoints[1]]/2;
    return out;
}

yarp::sig::Vector IndexCouplingHandler::decoupleRefTrq (yarp::sig::Vector& trq_ref)
{
    yarp::sig::Vector out =trq_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "IndexCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = trq_ref[m_coupledJoints[0]];
    out[m_coupledJoints[1]] = trq_ref[m_coupledJoints[1]]/2;
    out[m_coupledJoints[2]] = trq_ref[m_coupledJoints[1]]/2;
    return out;
}

//------------------------------------------------------------------------------------------------------------------
// MiddleCouplingHandler
//------------------------------------------------------------------------------------------------------------------

MiddleCouplingHandler::MiddleCouplingHandler(gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names, std::vector<Range> coupled_joint_limits)
: BaseCouplingHandler(model, coupled_joints,coupled_joint_names, coupled_joint_limits)
{
    m_couplingSize=3;
}

bool MiddleCouplingHandler::decouplePos(yarp::sig::Vector& current_pos)
{
    if (m_coupledJoints.size() != m_couplingSize) return false;
    current_pos[m_coupledJoints[1]] = current_pos[m_coupledJoints[1]] + current_pos[m_coupledJoints[2]];
    return true;
}

bool MiddleCouplingHandler::decoupleVel (yarp::sig::Vector& current_vel)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    current_vel[m_coupledJoints[1]] = current_vel[m_coupledJoints[1]] + current_vel[m_coupledJoints[2]];
    return true;
}

bool MiddleCouplingHandler::decoupleAcc (yarp::sig::Vector& current_acc)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    current_acc[m_coupledJoints[1]] = current_acc[m_coupledJoints[1]] + current_acc[m_coupledJoints[2]];
    return true;
}

bool MiddleCouplingHandler::decoupleTrq (yarp::sig::Vector& current_trq)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    return false;
}

yarp::sig::Vector MiddleCouplingHandler::decoupleRefPos (yarp::sig::Vector& pos_ref)
{
    yarp::sig::Vector out = pos_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "MiddleCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = pos_ref[m_coupledJoints[0]];
    out[m_coupledJoints[1]] = pos_ref[m_coupledJoints[1]]/2;
    out[m_coupledJoints[2]] = pos_ref[m_coupledJoints[1]]/2;
    return out;
}

yarp::sig::Vector MiddleCouplingHandler::decoupleRefVel (yarp::sig::Vector& vel_ref, const yarp::sig::Vector& pos_feedback)
{
    yarp::sig::Vector out = vel_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "MiddleCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = vel_ref[m_coupledJoints[0]];
    out[m_coupledJoints[1]] = vel_ref[m_coupledJoints[1]]/2;
    out[m_coupledJoints[2]] = vel_ref[m_coupledJoints[1]]/2;
    return out;
}

yarp::sig::Vector MiddleCouplingHandler::decoupleRefTrq (yarp::sig::Vector& trq_ref)
{
    yarp::sig::Vector out =trq_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "MiddleCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = trq_ref[m_coupledJoints[0]];
    out[m_coupledJoints[1]] = trq_ref[m_coupledJoints[1]]/2;
    out[m_coupledJoints[2]] = trq_ref[m_coupledJoints[1]]/2;
    return out;
}

//------------------------------------------------------------------------------------------------------------------
// PinkyCouplingHandler
//------------------------------------------------------------------------------------------------------------------

PinkyCouplingHandler::PinkyCouplingHandler(gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names, std::vector<Range> coupled_joint_limits)
: BaseCouplingHandler(model, coupled_joints,coupled_joint_names, coupled_joint_limits)
{
    m_couplingSize=6;
}

bool PinkyCouplingHandler::decouplePos (yarp::sig::Vector& current_pos)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    current_pos[m_coupledJoints[0]] = current_pos[m_coupledJoints[0]] + current_pos[m_coupledJoints[1]] + current_pos[m_coupledJoints[2]];
    return true;
}

bool PinkyCouplingHandler::decoupleVel (yarp::sig::Vector& current_vel)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    current_vel[m_coupledJoints[0]] = current_vel[m_coupledJoints[0]] + current_vel[m_coupledJoints[1]] + current_vel[m_coupledJoints[2]];
    return true;
}

bool PinkyCouplingHandler::decoupleAcc (yarp::sig::Vector& current_acc)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    current_acc[m_coupledJoints[0]] = current_acc[m_coupledJoints[0]] + current_acc[m_coupledJoints[1]] + current_acc[m_coupledJoints[2]];
    return true;
}

bool PinkyCouplingHandler::decoupleTrq (yarp::sig::Vector& current_trq)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    return false;
}

yarp::sig::Vector PinkyCouplingHandler::decoupleRefPos (yarp::sig::Vector& pos_ref)
{
    yarp::sig::Vector out = pos_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "PinkyCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = pos_ref[m_coupledJoints[0]]/3;
    out[m_coupledJoints[1]] = pos_ref[m_coupledJoints[0]]/3;
    out[m_coupledJoints[2]] = pos_ref[m_coupledJoints[0]]/3;
    out[m_coupledJoints[3]] = pos_ref[m_coupledJoints[0]]/3;
    out[m_coupledJoints[4]] = pos_ref[m_coupledJoints[0]]/3;
    out[m_coupledJoints[5]] = pos_ref[m_coupledJoints[0]]/3;
    return out;
}

yarp::sig::Vector PinkyCouplingHandler::decoupleRefVel (yarp::sig::Vector& vel_ref, const yarp::sig::Vector& pos_feedback)
{
    yarp::sig::Vector out = vel_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "PinkyCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = vel_ref[m_coupledJoints[0]]/3;
    out[m_coupledJoints[1]] = vel_ref[m_coupledJoints[0]]/3;
    out[m_coupledJoints[2]] = vel_ref[m_coupledJoints[0]]/3;
    out[m_coupledJoints[3]] = vel_ref[m_coupledJoints[0]]/3;
    out[m_coupledJoints[4]] = vel_ref[m_coupledJoints[0]]/3;
    out[m_coupledJoints[5]] = vel_ref[m_coupledJoints[0]]/3;
    return out;
}

yarp::sig::Vector PinkyCouplingHandler::decoupleRefTrq (yarp::sig::Vector& trq_ref)
{
    yarp::sig::Vector out =trq_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "PinkyCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = trq_ref[m_coupledJoints[0]]/3;
    out[m_coupledJoints[1]] = trq_ref[m_coupledJoints[0]]/3;
    out[m_coupledJoints[2]] = trq_ref[m_coupledJoints[0]]/3;
    out[m_coupledJoints[3]] = trq_ref[m_coupledJoints[0]]/3;
    out[m_coupledJoints[4]] = trq_ref[m_coupledJoints[0]]/3;
    out[m_coupledJoints[5]] = trq_ref[m_coupledJoints[0]]/3;
    return out;
}

//------------------------------------------------------------------------------------------------------------------
// FingersAbductionCouplingHandler
//------------------------------------------------------------------------------------------------------------------

FingersAbductionCouplingHandler::FingersAbductionCouplingHandler(gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names, std::vector<Range> coupled_joint_limits)
: BaseCouplingHandler(model, coupled_joints,coupled_joint_names, coupled_joint_limits)
{
    m_couplingSize = 4;
}

bool FingersAbductionCouplingHandler::decouplePos (yarp::sig::Vector& current_pos)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    current_pos[m_coupledJoints[0]] = (20.0 - current_pos[m_coupledJoints[2]])*3;
    return true;
}

bool FingersAbductionCouplingHandler::decoupleVel (yarp::sig::Vector& current_vel)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    current_vel[m_coupledJoints[0]] = -current_vel[m_coupledJoints[2]]*3;
    return true;
}

bool FingersAbductionCouplingHandler::decoupleAcc (yarp::sig::Vector& current_acc)
{

    if (m_coupledJoints.size()!=m_couplingSize) return false;
    current_acc[m_coupledJoints[0]] = -current_acc[m_coupledJoints[2]]*3;
    return true;
}

bool FingersAbductionCouplingHandler::decoupleTrq (yarp::sig::Vector& current_trq)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    return false;
}

yarp::sig::Vector FingersAbductionCouplingHandler::decoupleRefPos (yarp::sig::Vector& pos_ref)
{
    yarp::sig::Vector out = pos_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "FingersAbductionCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = -(20.0 - pos_ref[m_coupledJoints[0]]/3);
    out[m_coupledJoints[1]] = 0.0;
    out[m_coupledJoints[2]] = 20.0 - pos_ref[m_coupledJoints[0]]/3;
    out[m_coupledJoints[3]] = 20.0 - pos_ref[m_coupledJoints[0]]/3;
    return out;
}

yarp::sig::Vector FingersAbductionCouplingHandler::decoupleRefVel (yarp::sig::Vector& vel_ref, const yarp::sig::Vector& pos_feedback)
{
    yarp::sig::Vector out = vel_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "FingersAbductionCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = vel_ref[m_coupledJoints[0]]/3;
    out[m_coupledJoints[1]] = 0.0;
    out[m_coupledJoints[2]] = -vel_ref[m_coupledJoints[0]]/3;
    out[m_coupledJoints[3]] = -vel_ref[m_coupledJoints[0]]/3;
    return out;
}

yarp::sig::Vector FingersAbductionCouplingHandler::decoupleRefTrq (yarp::sig::Vector& trq_ref)
{
    yarp::sig::Vector out =trq_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "FingersAbductionCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = trq_ref[m_coupledJoints[0]]/3;
    out[m_coupledJoints[1]] = 0.0;
    out[m_coupledJoints[2]] = -trq_ref[m_coupledJoints[0]]/3;
    out[m_coupledJoints[3]] = -trq_ref[m_coupledJoints[0]]/3;
    return out;
}

//------------------------------------------------------------------------------------------------------------------
// CerHandCouplingHandler
//------------------------------------------------------------------------------------------------------------------

CerHandCouplingHandler::CerHandCouplingHandler(gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names, std::vector<Range> coupled_joint_limits)
: BaseCouplingHandler(model, coupled_joints,coupled_joint_names, coupled_joint_limits)
{
    m_couplingSize = 4;
}

bool CerHandCouplingHandler::decouplePos (yarp::sig::Vector& current_pos)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    double temp0 = current_pos[m_coupledJoints[0]];
    double temp2 = current_pos[m_coupledJoints[2]];
    current_pos[m_coupledJoints[0]] = temp0 + current_pos[m_coupledJoints[1]];
    current_pos[m_coupledJoints[1]] = temp2 + current_pos[m_coupledJoints[3]];
    return true;
}

bool CerHandCouplingHandler::decoupleVel (yarp::sig::Vector& current_vel)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    double temp0 = current_vel[m_coupledJoints[0]];
    double temp2 = current_vel[m_coupledJoints[2]];
    current_vel[m_coupledJoints[0]] = temp0 + current_vel[m_coupledJoints[1]];
    current_vel[m_coupledJoints[1]] = temp2 + current_vel[m_coupledJoints[3]];
    return true;
}

bool CerHandCouplingHandler::decoupleAcc (yarp::sig::Vector& current_acc)
{

    if (m_coupledJoints.size()!=m_couplingSize) return false;
    double temp0 = current_acc[m_coupledJoints[0]];
    double temp2 = current_acc[m_coupledJoints[2]];
    current_acc[m_coupledJoints[0]] = temp0 + current_acc[m_coupledJoints[1]];
    current_acc[m_coupledJoints[1]] = temp2 + current_acc[m_coupledJoints[3]];
    return true;
}

bool CerHandCouplingHandler::decoupleTrq (yarp::sig::Vector& current_trq)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    return false;
}

yarp::sig::Vector CerHandCouplingHandler::decoupleRefPos (yarp::sig::Vector& pos_ref)
{
    yarp::sig::Vector out = pos_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "CerHandCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = pos_ref[m_coupledJoints[0]]/2;
    out[m_coupledJoints[1]] = pos_ref[m_coupledJoints[0]]/2;
    out[m_coupledJoints[2]] = pos_ref[m_coupledJoints[1]]/2;
    out[m_coupledJoints[3]] = pos_ref[m_coupledJoints[1]]/2;
    return out;
}

yarp::sig::Vector CerHandCouplingHandler::decoupleRefVel (yarp::sig::Vector& vel_ref, const yarp::sig::Vector& pos_feedback)
{
    yarp::sig::Vector out = vel_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "CerHandCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = vel_ref[m_coupledJoints[0]]/2;
    out[m_coupledJoints[1]] = vel_ref[m_coupledJoints[0]]/2;
    out[m_coupledJoints[2]] = vel_ref[m_coupledJoints[1]]/2;
    out[m_coupledJoints[3]] = vel_ref[m_coupledJoints[1]]/2;
    return out;
}

yarp::sig::Vector CerHandCouplingHandler::decoupleRefTrq (yarp::sig::Vector& trq_ref)
{
    yarp::sig::Vector out =trq_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "CerHandCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = trq_ref[m_coupledJoints[0]]/2;
    out[m_coupledJoints[1]] = trq_ref[m_coupledJoints[0]]/2;
    out[m_coupledJoints[2]] = trq_ref[m_coupledJoints[1]]/2;
    out[m_coupledJoints[3]] = trq_ref[m_coupledJoints[1]]/2;
    return out;
}

//------------------------------------------------------------------------------------------------------------------
// HandMk3CouplingHandler
//------------------------------------------------------------------------------------------------------------------

HandMk3CouplingHandler::HandMk3CouplingHandler(gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names, std::vector<Range> coupled_joint_limits)
: BaseCouplingHandler(model, coupled_joints,coupled_joint_names, coupled_joint_limits), LUTSIZE(4096)
{
    const double RAD2DEG = 180.0/atan2(0.0,-1.0);
    const double DEG2RAD = 1.0/RAD2DEG;

    m_couplingSize = 11;

    thumb_lut.resize(LUTSIZE);
    index_lut.resize(LUTSIZE);

    std::vector<double> num(LUTSIZE);

    for (int n = 0; n < LUTSIZE; ++n)
    {
        num[n] = 0.0;
        thumb_lut[n] = 0.0;
    }

    // thumb
    {
        double L0x = -0.00555,          L0y = 0.00195+0.0009;
        double P1x =  0.020,            P1y = 0.0015;
        double L1x =  P1x-0.0055-0.003, L1y = P1y-0.002+0.0019;

        double l2 = (P1x - L1x)*(P1x - L1x) + (P1y - L1y)*(P1y - L1y);
        double k2 = (L1x - L0x)*(L1x - L0x) + (L1y - L0y)*(L1y - L0y);

        double offset = RAD2DEG*atan2(L1y-P1y, L1x-P1x);

        for (double q1 = 0.0; q1 <= 85.5; q1 += 0.01)
        {
            double cq1 = cos(DEG2RAD*q1);
            double sq1 = sin(DEG2RAD*q1);

            double P1xr = cq1*P1x-sq1*P1y;
            double P1yr = sq1*P1x+cq1*P1y;

            double h2 = (P1xr - L0x)*(P1xr - L0x) + (P1yr - L0y)*(P1yr - L0y);

            double alfa = RAD2DEG*atan2(L0y - P1yr, L0x - P1xr);

            double beta = RAD2DEG*acos((h2 + l2 - k2)/(2.0*sqrt(h2*l2)));

            double q2 = alfa + beta - offset;

            while (q2 <    0.0) q2 += 360.0;
            while (q2 >= 360.0) q2 -= 360.0;

            double dindex = q2*10.0;

            int iindex = int(dindex);

            double w = dindex - double(iindex);

            thumb_lut[iindex] += (1.0 - w)*q1;
            num[iindex] += (1.0 - w);

            thumb_lut[iindex + 1] += w*q1;
            num[iindex + 1] += w;
        }

        for (int n = 0; n < LUTSIZE; ++n)
        {
            if (num[n] > 0.0)
            {
                thumb_lut[n] /= num[n];
            }
        }
    }

    for (int n = 0; n < LUTSIZE; ++n)
    {
        num[n] = 0.0;
        index_lut[n] = 0.0;
    }

    // finger
    {
        double P1x =  0.0300, P1y = 0.0015;
        double L0x = -0.0050, L0y = 0.0040;
        double L1x =  0.0240, L1y = 0.0008;

        double l2 = (P1x - L1x)*(P1x - L1x) + (P1y - L1y)*(P1y - L1y);
        double k2 = (L1x - L0x)*(L1x - L0x) + (L1y - L0y)*(L1y - L0y);

        double offset = RAD2DEG*atan2(L1y-P1y, L1x-P1x);

        for (double q1 = 0.0; q1 <= 95.5; q1 += 0.01)
        {
            double cq1 = cos(DEG2RAD*q1);
            double sq1 = sin(DEG2RAD*q1);

            double P1xr = cq1*P1x-sq1*P1y;
            double P1yr = sq1*P1x+cq1*P1y;

            double h2 = (P1xr - L0x)*(P1xr - L0x) + (P1yr - L0y)*(P1yr - L0y);

            double alfa = RAD2DEG*atan2(L0y - P1yr, L0x - P1xr);

            double beta = RAD2DEG*acos((h2 + l2 - k2)/(2.0*sqrt(h2*l2)));

            double q2 = alfa + beta - offset;

            while (q2 <    0.0) q2 += 360.0;
            while (q2 >= 360.0) q2 -= 360.0;

            // get decimal part of q2 to find out how to weigh index and index+1
            double dindex = q2*10.0;
            int iindex = int(dindex);
            double w = dindex - double(iindex);

            // Construct LUT
            index_lut[iindex] += (1.0 - w)*q1;
            num[iindex] += (1.0 - w);

            index_lut[iindex + 1] += w*q1;
            num[iindex + 1] += w;
        }

        // divide each value in the LUT by the weight if it is greater than 0 to extract q1
        for (int n = 0; n < LUTSIZE; ++n)
        {
            if (num[n] > 0.0)
            {
                index_lut[n] /= num[n];
            }
        }
    }
}

bool HandMk3CouplingHandler::decouplePos (yarp::sig::Vector& current_pos)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    current_pos[m_coupledJoints[2]] = current_pos[m_coupledJoints[2]] + current_pos[m_coupledJoints[3]];
    current_pos[m_coupledJoints[3]] = current_pos[m_coupledJoints[4]];
    current_pos[m_coupledJoints[4]] = current_pos[m_coupledJoints[5]] + current_pos[m_coupledJoints[6]];
    current_pos[m_coupledJoints[5]] = current_pos[m_coupledJoints[7]] + current_pos[m_coupledJoints[8]];
    current_pos[m_coupledJoints[6]] = current_pos[m_coupledJoints[9]] + current_pos[m_coupledJoints[10]];
    return true;
}

bool HandMk3CouplingHandler::decoupleVel (yarp::sig::Vector& current_vel)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    current_vel[m_coupledJoints[2]] = current_vel[m_coupledJoints[2]] + current_vel[m_coupledJoints[3]];
    current_vel[m_coupledJoints[3]] = current_vel[m_coupledJoints[4]];
    current_vel[m_coupledJoints[4]] = current_vel[m_coupledJoints[5]] + current_vel[m_coupledJoints[6]];
    current_vel[m_coupledJoints[5]] = current_vel[m_coupledJoints[7]] + current_vel[m_coupledJoints[8]];
    current_vel[m_coupledJoints[6]] = current_vel[m_coupledJoints[9]] + current_vel[m_coupledJoints[10]];
    return true;
}

bool HandMk3CouplingHandler::decoupleAcc (yarp::sig::Vector& current_acc)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    current_acc[m_coupledJoints[2]] = current_acc[m_coupledJoints[2]] + current_acc[m_coupledJoints[3]];
    current_acc[m_coupledJoints[3]] = current_acc[m_coupledJoints[4]];
    current_acc[m_coupledJoints[4]] = current_acc[m_coupledJoints[5]] + current_acc[m_coupledJoints[6]];
    current_acc[m_coupledJoints[5]] = current_acc[m_coupledJoints[7]] + current_acc[m_coupledJoints[8]];
    current_acc[m_coupledJoints[6]] = current_acc[m_coupledJoints[9]] + current_acc[m_coupledJoints[10]];
    return true;
}

bool HandMk3CouplingHandler::decoupleTrq (yarp::sig::Vector& current_trq)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    return false;
}

double HandMk3CouplingHandler::decouple (double q2, std::vector<double>& lut)
{

    double dindex = q2*10.0;
    int iindex = int(dindex);
    // get decimal part of q2 to find out how to weigh index and index+1
    double w = dindex - double(iindex);
    // interpolate between index and the next with a convex combination weighting
    return lut[iindex]*(1.0 - w) + lut[iindex + 1]*w;
}

yarp::sig::Vector HandMk3CouplingHandler::decoupleRefPos (yarp::sig::Vector& pos_ref)
{
    yarp::sig::Vector out = pos_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "HandMk3CouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[2]]  = decouple(pos_ref[m_coupledJoints[2]], thumb_lut);
    out[m_coupledJoints[3]]  = pos_ref[m_coupledJoints[2]] - out[m_coupledJoints[2]];
    out[m_coupledJoints[4]]  = pos_ref[m_coupledJoints[3]];
    out[m_coupledJoints[5]]  = decouple(pos_ref[m_coupledJoints[4]], index_lut);
    out[m_coupledJoints[6]]  = pos_ref[m_coupledJoints[4]] - out[m_coupledJoints[5]];
    out[m_coupledJoints[7]]  = decouple(pos_ref[m_coupledJoints[5]], index_lut);
    out[m_coupledJoints[8]]  = pos_ref[m_coupledJoints[5]] - out[m_coupledJoints[7]];
    out[m_coupledJoints[9]]  = decouple(pos_ref[m_coupledJoints[6]], index_lut);
    out[m_coupledJoints[10]] = pos_ref[m_coupledJoints[6]] - out[m_coupledJoints[9]];
    return out;
}

yarp::sig::Vector HandMk3CouplingHandler::decoupleRefVel (yarp::sig::Vector& vel_ref, const yarp::sig::Vector& pos_feedback)
{
    yarp::sig::Vector out = vel_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "HandMk3CouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[2]]  = decouple(vel_ref[m_coupledJoints[2]], thumb_lut);
    out[m_coupledJoints[3]]  = vel_ref[m_coupledJoints[2]] - out[m_coupledJoints[2]];
    out[m_coupledJoints[4]]  = vel_ref[m_coupledJoints[3]];
    out[m_coupledJoints[5]]  = decouple(vel_ref[m_coupledJoints[4]], index_lut);
    out[m_coupledJoints[6]]  = vel_ref[m_coupledJoints[4]] - out[m_coupledJoints[5]];
    out[m_coupledJoints[7]]  = decouple(vel_ref[m_coupledJoints[5]], index_lut);
    out[m_coupledJoints[8]]  = vel_ref[m_coupledJoints[5]] - out[m_coupledJoints[7]];
    out[m_coupledJoints[9]]  = decouple(vel_ref[m_coupledJoints[6]], index_lut);
    out[m_coupledJoints[10]] = vel_ref[m_coupledJoints[6]] - out[m_coupledJoints[9]];
    return out;
}

yarp::sig::Vector HandMk3CouplingHandler::decoupleRefTrq (yarp::sig::Vector& trq_ref)
{
    yarp::sig::Vector out =trq_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "HandMk3CouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[2]]  = decouple(trq_ref[m_coupledJoints[2]], thumb_lut);
    out[m_coupledJoints[3]]  = trq_ref[m_coupledJoints[2]] - out[m_coupledJoints[2]];
    out[m_coupledJoints[4]]  = trq_ref[m_coupledJoints[3]];
    out[m_coupledJoints[5]]  = decouple(trq_ref[m_coupledJoints[4]], index_lut);
    out[m_coupledJoints[6]]  = trq_ref[m_coupledJoints[4]] - out[m_coupledJoints[5]];
    out[m_coupledJoints[7]]  = decouple(trq_ref[m_coupledJoints[5]], index_lut);
    out[m_coupledJoints[8]]  = trq_ref[m_coupledJoints[5]] - out[m_coupledJoints[7]];
    out[m_coupledJoints[9]]  = decouple(trq_ref[m_coupledJoints[6]], index_lut);
    out[m_coupledJoints[10]] = trq_ref[m_coupledJoints[6]] - out[m_coupledJoints[9]];
    return out;
}

//------------------------------------------------------------------------------------------------------------------
// HandMk4CouplingHandler
//------------------------------------------------------------------------------------------------------------------

HandMk4CouplingHandler::HandMk4CouplingHandler(gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names, std::vector<Range> coupled_joint_limits)
: BaseCouplingHandler(model, coupled_joints,coupled_joint_names, coupled_joint_limits), LUTSIZE(4096)
{
    const double RAD2DEG = 180.0/atan2(0.0,-1.0);
    const double DEG2RAD = 1.0/RAD2DEG;

    m_couplingSize = 13;

    thumb_lut.resize(LUTSIZE);
    index_lut.resize(LUTSIZE);
    pinkie_lut.resize(LUTSIZE);

    std::vector<double> num(LUTSIZE);

    // thumb
    for (int n = 0; n < LUTSIZE; ++n)
    {
        num[n] = 0.0;
        thumb_lut[n] = 0.0;
    }
    {
        double L0x = -0.00555,    L0y = 0.00285;
        double P1x =  0.02,       P1y = 0.0015;
        double L1x =  0.0115,     L1y = 0.0015;

        double l2 = (P1x - L1x)*(P1x - L1x) + (P1y - L1y)*(P1y - L1y);
        double k2 = (L1x - L0x)*(L1x - L0x) + (L1y - L0y)*(L1y - L0y);

        double offset = 180;

        for (double q1 = 0.0; q1 <= 85.5; q1 += 0.01)
        {
            double cq1 = cos(DEG2RAD*q1);
            double sq1 = sin(DEG2RAD*q1);

            double P1xr = cq1*P1x-sq1*P1y;
            double P1yr = sq1*P1x+cq1*P1y;

            double h2 = (P1xr - L0x)*(P1xr - L0x) + (P1yr - L0y)*(P1yr - L0y);

            double alfa = RAD2DEG*atan2(L0y - P1yr, L0x - P1xr);

            double beta = RAD2DEG*acos((h2 + l2 - k2)/(2.0*sqrt(h2*l2)));

            double q2 = alfa + beta - offset;

            while (q2 <    0.0) q2 += 360.0;
            while (q2 >= 360.0) q2 -= 360.0;

            double dindex = q2*10.0;

            int iindex = int(dindex);

            double w = dindex - double(iindex);

            thumb_lut[iindex] += (1.0 - w)*q1;
            num[iindex] += (1.0 - w);

            thumb_lut[iindex + 1] += w*q1;
            num[iindex + 1] += w;
        }

        for (int n = 0; n < LUTSIZE; ++n)
        {
            if (num[n] > 0.0)
            {
                thumb_lut[n] /= num[n];
            }
        }
    }

    // index, middle, ring
    for (int n = 0; n < LUTSIZE; ++n)
    {
        num[n] = 0.0;
        index_lut[n] = 0.0;
    }
    {
        double P1x =  0.0300, P1y = 0.0015;
        double L0x = -0.0050, L0y = 0.0040;
        double L1x =  0.0240, L1y = 0.0008;

        double l2 = (P1x - L1x)*(P1x - L1x) + (P1y - L1y)*(P1y - L1y);
        double k2 = (L1x - L0x)*(L1x - L0x) + (L1y - L0y)*(L1y - L0y);

        double offset = 173.35;

        for (double q1 = 0.0; q1 <= 95.5; q1 += 0.01)
        {
            double cq1 = cos(DEG2RAD*q1);
            double sq1 = sin(DEG2RAD*q1);

            double P1xr = cq1*P1x-sq1*P1y;
            double P1yr = sq1*P1x+cq1*P1y;

            double h2 = (P1xr - L0x)*(P1xr - L0x) + (P1yr - L0y)*(P1yr - L0y);

            double alfa = RAD2DEG*atan2(L0y - P1yr, L0x - P1xr);

            double beta = RAD2DEG*acos((h2 + l2 - k2)/(2.0*sqrt(h2*l2)));

            double q2 = alfa + beta - offset;

            while (q2 <    0.0) q2 += 360.0;
            while (q2 >= 360.0) q2 -= 360.0;

            // get decimal part of q2 to find out how to weigh index and index+1
            double dindex = q2*10.0;
            int iindex = int(dindex);
            double w = dindex - double(iindex);

            // Construct LUT
            index_lut[iindex] += (1.0 - w)*q1;
            num[iindex] += (1.0 - w);

            index_lut[iindex + 1] += w*q1;
            num[iindex + 1] += w;
        }

        // divide each value in the LUT by the weight if it is greater than 0 to extract q1
        for (int n = 0; n < LUTSIZE; ++n)
        {
            if (num[n] > 0.0)
            {
                index_lut[n] /= num[n];
            }
        }
    }

    // pinkie
    for (int n = 0; n < LUTSIZE; ++n)
    {
        num[n] = 0.0;
        pinkie_lut[n] = 0.0;
    }
    {
        double P1x =  0.0250, P1y = 0.0015;
        double L0x = -0.0050, L0y = 0.0040;
        double L1x =  0.0190, L1y = 0.0005;

        double l2 = (P1x - L1x)*(P1x - L1x) + (P1y - L1y)*(P1y - L1y);
        double k2 = (L1x - L0x)*(L1x - L0x) + (L1y - L0y)*(L1y - L0y);

        double offset = 170.54;
        for (double q1 = 0.0; q1 <= 95.5; q1 += 0.01)
        {
            double cq1 = cos(DEG2RAD*q1);
            double sq1 = sin(DEG2RAD*q1);

            double P1xr = cq1*P1x-sq1*P1y;
            double P1yr = sq1*P1x+cq1*P1y;

            double h2 = (P1xr - L0x)*(P1xr - L0x) + (P1yr - L0y)*(P1yr - L0y);

            double alfa = RAD2DEG*atan2(L0y - P1yr, L0x - P1xr);

            double beta = RAD2DEG*acos((h2 + l2 - k2)/(2.0*sqrt(h2*l2)));

            double q2 = alfa + beta - offset;

            while (q2 <    0.0) q2 += 360.0;
            while (q2 >= 360.0) q2 -= 360.0;

            // get decimal part of q2 to find out how to weigh index and index+1
            double dindex = q2*10.0;
            int iindex = int(dindex);
            double w = dindex - double(iindex);

            // Construct LUT
            pinkie_lut[iindex] += (1.0 - w)*q1;
            num[iindex] += (1.0 - w);

            pinkie_lut[iindex + 1] += w*q1;
            num[iindex + 1] += w;
        }

        // divide each value in the LUT by the weight if it is greater than 0 to extract q1
        for (int n = 0; n < LUTSIZE; ++n)
        {
            if (num[n] > 0.0)
            {
                pinkie_lut[n] /= num[n];
            }
        }
    }
}

bool HandMk4CouplingHandler::decouplePos (yarp::sig::Vector& current_pos)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    current_pos[m_coupledJoints[2]] = current_pos[m_coupledJoints[2]] + current_pos[m_coupledJoints[3]];
    current_pos[m_coupledJoints[3]] = current_pos[m_coupledJoints[4]];
    current_pos[m_coupledJoints[4]] = current_pos[m_coupledJoints[5]] + current_pos[m_coupledJoints[6]];
    current_pos[m_coupledJoints[5]] = current_pos[m_coupledJoints[7]] + current_pos[m_coupledJoints[8]];
    current_pos[m_coupledJoints[6]] = current_pos[m_coupledJoints[9]] + current_pos[m_coupledJoints[10]];
    return true;
}

bool HandMk4CouplingHandler::decoupleVel (yarp::sig::Vector& current_vel)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    current_vel[m_coupledJoints[2]] = current_vel[m_coupledJoints[2]] + current_vel[m_coupledJoints[3]];
    current_vel[m_coupledJoints[3]] = current_vel[m_coupledJoints[4]];
    current_vel[m_coupledJoints[4]] = current_vel[m_coupledJoints[5]] + current_vel[m_coupledJoints[6]];
    current_vel[m_coupledJoints[5]] = current_vel[m_coupledJoints[7]] + current_vel[m_coupledJoints[8]];
    current_vel[m_coupledJoints[6]] = current_vel[m_coupledJoints[9]] + current_vel[m_coupledJoints[10]];
    return true;
}

bool HandMk4CouplingHandler::decoupleAcc (yarp::sig::Vector& current_acc)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    current_acc[m_coupledJoints[2]] = current_acc[m_coupledJoints[2]] + current_acc[m_coupledJoints[3]];
    current_acc[m_coupledJoints[3]] = current_acc[m_coupledJoints[4]];
    current_acc[m_coupledJoints[4]] = current_acc[m_coupledJoints[5]] + current_acc[m_coupledJoints[6]];
    current_acc[m_coupledJoints[5]] = current_acc[m_coupledJoints[7]] + current_acc[m_coupledJoints[8]];
    current_acc[m_coupledJoints[6]] = current_acc[m_coupledJoints[9]] + current_acc[m_coupledJoints[10]];
    return true;
}

bool HandMk4CouplingHandler::decoupleTrq (yarp::sig::Vector& current_trq)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    return false;
}

double HandMk4CouplingHandler::decouple (double q2, std::vector<double>& lut)
{

    double dindex = q2*10.0;
    int iindex = int(dindex);
    // get decimal part of q2 to find out how to weigh index and index+1
    double w = dindex - double(iindex);
    // interpolate between index and the next with a convex combination weighting
    return lut[iindex]*(1.0 - w) + lut[iindex + 1]*w;
}

yarp::sig::Vector HandMk4CouplingHandler::decoupleRefPos (yarp::sig::Vector& pos_ref)
{
    yarp::sig::Vector out = pos_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "HandMk4CouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[2]]  = decouple(pos_ref[m_coupledJoints[2]], thumb_lut);
    out[m_coupledJoints[3]]  = pos_ref[m_coupledJoints[2]] - out[m_coupledJoints[2]];
    out[m_coupledJoints[4]]  = pos_ref[m_coupledJoints[3]];
    out[m_coupledJoints[5]]  = decouple(pos_ref[m_coupledJoints[4]], index_lut);
    out[m_coupledJoints[6]]  = pos_ref[m_coupledJoints[4]] - out[m_coupledJoints[5]];
    out[m_coupledJoints[7]]  = decouple(pos_ref[m_coupledJoints[5]], index_lut);
    out[m_coupledJoints[8]]  = pos_ref[m_coupledJoints[5]] - out[m_coupledJoints[7]];
    out[m_coupledJoints[9]]  = decouple(pos_ref[m_coupledJoints[6]], index_lut);
    out[m_coupledJoints[10]] = pos_ref[m_coupledJoints[6]] - out[m_coupledJoints[9]];
    out[m_coupledJoints[11]] = decouple(pos_ref[m_coupledJoints[6]], pinkie_lut);;
    out[m_coupledJoints[12]] = pos_ref[m_coupledJoints[6]] - out[m_coupledJoints[11]];
    return out;
}

yarp::sig::Vector HandMk4CouplingHandler::decoupleRefVel (yarp::sig::Vector& vel_ref, const yarp::sig::Vector& pos_feedback)
{
    yarp::sig::Vector out = vel_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "HandMk4CouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[2]]  = decouple(vel_ref[m_coupledJoints[2]], thumb_lut);
    out[m_coupledJoints[3]]  = vel_ref[m_coupledJoints[2]] - out[m_coupledJoints[2]];
    out[m_coupledJoints[4]]  = vel_ref[m_coupledJoints[3]];
    out[m_coupledJoints[5]]  = decouple(vel_ref[m_coupledJoints[4]], index_lut);
    out[m_coupledJoints[6]]  = vel_ref[m_coupledJoints[4]] - out[m_coupledJoints[5]];
    out[m_coupledJoints[7]]  = decouple(vel_ref[m_coupledJoints[5]], index_lut);
    out[m_coupledJoints[8]]  = vel_ref[m_coupledJoints[5]] - out[m_coupledJoints[7]];
    out[m_coupledJoints[9]]  = decouple(vel_ref[m_coupledJoints[6]], index_lut);
    out[m_coupledJoints[10]] = vel_ref[m_coupledJoints[6]] - out[m_coupledJoints[9]];
    out[m_coupledJoints[11]] = decouple(vel_ref[m_coupledJoints[6]], pinkie_lut);;
    out[m_coupledJoints[12]] = vel_ref[m_coupledJoints[6]] - out[m_coupledJoints[11]];
    return out;
}

yarp::sig::Vector HandMk4CouplingHandler::decoupleRefTrq (yarp::sig::Vector& trq_ref)
{
    yarp::sig::Vector out =trq_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "HandMk4CouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[2]]  = decouple(trq_ref[m_coupledJoints[2]], thumb_lut);
    out[m_coupledJoints[3]]  = trq_ref[m_coupledJoints[2]] - out[m_coupledJoints[2]];
    out[m_coupledJoints[4]]  = trq_ref[m_coupledJoints[3]];
    out[m_coupledJoints[5]]  = decouple(trq_ref[m_coupledJoints[4]], index_lut);
    out[m_coupledJoints[6]]  = trq_ref[m_coupledJoints[4]] - out[m_coupledJoints[5]];
    out[m_coupledJoints[7]]  = decouple(trq_ref[m_coupledJoints[5]], index_lut);
    out[m_coupledJoints[8]]  = trq_ref[m_coupledJoints[5]] - out[m_coupledJoints[7]];
    out[m_coupledJoints[9]]  = decouple(trq_ref[m_coupledJoints[6]], index_lut);
    out[m_coupledJoints[10]] = trq_ref[m_coupledJoints[6]] - out[m_coupledJoints[9]];
    out[m_coupledJoints[11]] = decouple(trq_ref[m_coupledJoints[6]], pinkie_lut);;
    out[m_coupledJoints[12]] = trq_ref[m_coupledJoints[6]] - out[m_coupledJoints[11]];
    return out;
}

//------------------------------------------------------------------------------------------------------------------
// HandMk5CouplingHandler
//------------------------------------------------------------------------------------------------------------------

HandMk5CouplingHandler::HandMk5CouplingHandler(gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names, std::vector<Range> coupled_joint_limits)
: BaseCouplingHandler(model, coupled_joints,coupled_joint_names, coupled_joint_limits)
{
    m_couplingSize = 12;
}

bool HandMk5CouplingHandler::parseFingerParameters(yarp::os::Bottle& hand_params)
{
    auto L0x    = hand_params.findGroup("L0x");
    auto L0y    = hand_params.findGroup("L0y");
    auto q2bias = hand_params.findGroup("q2bias");
    auto q1off  = hand_params.findGroup("q1off");
    auto k      = hand_params.findGroup("k");
    auto d      = hand_params.findGroup("d");
    auto l      = hand_params.findGroup("l");
    auto b      = hand_params.findGroup("b");

    constexpr int nFingers = 5;
    // All the +1 is because the first element of the bottle is the name of the group
    if(L0x.size()!=nFingers+1 || L0y.size()!=nFingers+1 || q2bias.size()!=nFingers+1 ||
       q1off.size()!=nFingers+1 || k.size()!=nFingers+1 || d.size()!=nFingers+1 ||
       l.size()!=nFingers+1 || b.size()!=nFingers+1 )
    {
        yError()<<"HandMk5CouplingHandler: invalid hand parameters, check your configuration file";
        return false;
    }


    const std::array<std::string,5> names = {"thumb", "index", "middle", "ring", "pinky"};
    for (std::size_t i = 0; i < names.size(); i++)
    {
        mFingerParameters.insert({names.at(i), {L0x.get(i+1).asFloat32(), L0y.get(i+1).asFloat32(), q2bias.get(i+1).asFloat32(),
                                  q1off.get(i+1).asFloat32(), k.get(i+1).asFloat32(), d.get(i+1).asFloat32(),
                                  l.get(i+1).asFloat32(), b.get(i+1).asFloat32()}});
    }

    return true;
}

bool HandMk5CouplingHandler::decouplePos (yarp::sig::Vector& current_pos)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;

    const yarp::sig::Vector tmp = current_pos;

    /* thumb_add <-- thumb_add */
    current_pos[m_coupledJoints[0]] = tmp[0];
    /* thumb_oc <-- thumb_prox */
    current_pos[m_coupledJoints[1]] = tmp[1];
    /* index_add <-- index_add */
    current_pos[m_coupledJoints[2]] = tmp[3];
    /* index_oc <-- index_prox */
    current_pos[m_coupledJoints[3]] = tmp[4];
    /* middle_oc <-- middle_prox */
    current_pos[m_coupledJoints[4]] = tmp[6];
    /**
     * ring_pinky_oc <-- pinkie_prox
     * as, on the real robot, the coupled group composed of ring_prox, ring_dist, pinkie_prox and pinkie_dist
     * is controlled using the encoder on the pinkie_prox as feedback
     */
    current_pos[m_coupledJoints[5]] = tmp[10];

    return true;
}

bool HandMk5CouplingHandler::decoupleVel (yarp::sig::Vector& current_vel)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;

    const yarp::sig::Vector tmp = current_vel;

    /* thumb_add <-- thumb_add */
    current_vel[m_coupledJoints[0]] = tmp[0];
    /* thumb_oc <-- thumb_prox */
    current_vel[m_coupledJoints[1]] = tmp[1];
    /* index_add <-- index_add */
    current_vel[m_coupledJoints[2]] = tmp[3];
    /* index_oc <-- index_prox */
    current_vel[m_coupledJoints[3]] = tmp[4];
    /* middle_oc <-- middle_prox */
    current_vel[m_coupledJoints[4]] = tmp[6];
    /**
    * ring_pinky_oc <-- pinkie_prox
    * as, on the real robot, the coupled group composed of ring_prox, ring_dist, pinkie_prox and pinkie_dist
    * is controlled using the encoder on the pinkie_prox as feedback
    */
    current_vel[m_coupledJoints[5]] = tmp[10];

    return true;
}

bool HandMk5CouplingHandler::decoupleAcc (yarp::sig::Vector& current_acc)
{
    /**
     * Acceleration control not available for fingers on the real robot.
     * Note: this method is never called within the controlboard plugin code.
     */
    return false;
}

bool HandMk5CouplingHandler::decoupleTrq (yarp::sig::Vector& current_trq)
{
    /**
     * Torque control not available for fingers on the real robot.
     */
    return false;
}

yarp::sig::Vector HandMk5CouplingHandler::decoupleRefPos (yarp::sig::Vector& pos_ref)
{
    if (m_coupledJoints.size()!=m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "HandMk5CouplingHandler: Invalid coupling vector"; return pos_ref;}

    yarp::sig::Vector out(pos_ref.size());

    /* thumb_add <-- thumb_add */
    out[0] = pos_ref[m_coupledJoints[0]];
    /* thumb_prox <-- thumb_oc */
    out[1] = pos_ref[m_coupledJoints[1]];
    /* thumb_dist <-- coupling_law(thumb_prox) */
    out[2] = evaluateCoupledJoint(out[1], "thumb");
    /* index_add <-- index_add */
    out[3] = pos_ref[m_coupledJoints[2]];
    /* index_prox <-- index_oc */
    out[4] = pos_ref[m_coupledJoints[3]];
    /* index_dist <-- coupling_law(index_prox) */
    out[5] = evaluateCoupledJoint(out[4], "index");
    /* middle_prox <-- middle_oc */
    out[6] = pos_ref[m_coupledJoints[4]];
    /* middle_dist <-- coupling_law(middle_prox) */
    out[7] = evaluateCoupledJoint(out[6], "middle");
    /* ring_prox <-- ring_pinky_oc */
    out[8] = pos_ref[m_coupledJoints[5]];
    /* ring_dist <-- coupling_law(ring_prox) */
    out[9] = evaluateCoupledJoint(out[8], "ring");
    /* pinky_prox <-- ring_pinky_oc */
    out[10] = pos_ref[m_coupledJoints[5]];
    /* pinky_dist <-- coupling_law(pinky_prox) */
    out[11] = evaluateCoupledJoint(out[10], "pinky");

    return out;
}


yarp::sig::Vector HandMk5CouplingHandler::decoupleRefVel (yarp::sig::Vector& vel_ref, const yarp::sig::Vector& pos_feedback)
{
    if (m_coupledJoints.size()!=m_couplingSize) {yCError(GAZEBOCONTROLBOARD) << "HandMk5CouplingHandler: Invalid coupling vector"; return vel_ref;}

    /**
     * Extract the current position of proximal joints from pos_feedback.
     */
    double lastThumbProx = pos_feedback[1];
    double lastIndexProx = pos_feedback[4];
    double lastMiddleProx = pos_feedback[6];
    double lastRingProx = pos_feedback[8];
    double lastPinkyProx = pos_feedback[10];

    /**
     * In the following, we use the fact that:
     * /dot{distal_joint} = \partial{distal_joint}{proximal_joint} \dot{proximal_joint}.
     */

    yarp::sig::Vector out(vel_ref.size());

    /* thumb_add <-- thumb_add */
    out[0] = vel_ref[m_coupledJoints[0]];
    /* thumb_prox <-- thumb_oc */
    out[1] = vel_ref[m_coupledJoints[1]];
    /* thumb_dist <-- coupling_law_jacobian(thumb_prox_position) * thumb_prox */
    out[2] = evaluateCoupledJointJacobian(lastThumbProx, "thumb") * out[1];
    /* index_add <-- index_add */
    out[3] = vel_ref[m_coupledJoints[2]];
    /* index_prox <-- index_oc */
    out[4] = vel_ref[m_coupledJoints[3]];
    /* index_dist <-- coupling_law_jacobian(index_prox_position) * index_prox */
    out[5] = evaluateCoupledJointJacobian(lastIndexProx, "index") * out[4];
    /* middle_prox <-- middle_oc */
    out[6] = vel_ref[m_coupledJoints[4]];
    /* middle_dist <-- coupling_law_jacobian(middle_prox_position) * middle_prox */
    out[7] = evaluateCoupledJointJacobian(lastMiddleProx, "middle") * out[6];
    /* ring_prox <-- ring_pinky_oc */
    out[8] = vel_ref[m_coupledJoints[5]];
    /* ring_dist <-- coupling_law_jacobian(ring_prox_position) * ring_prox */
    out[9] = evaluateCoupledJointJacobian(lastRingProx, "ring") * out[8];
    /* pinky_prox <-- ring_pinky_oc */
    out[10] = vel_ref[m_coupledJoints[5]];
    /* pinky_dist <-- coupling_law(pinky_prox) */
    out[11] = evaluateCoupledJointJacobian(lastPinkyProx, "pinky") * out[10];

    return out;
}

yarp::sig::Vector HandMk5CouplingHandler::decoupleRefTrq (yarp::sig::Vector& trq_ref)
{
    /**
     * Torque control not available for fingers on the real robot.
     */
    return trq_ref;
}

double HandMk5CouplingHandler::evaluateCoupledJoint(const double& q1, const std::string& finger_name)
{
    /**
     * Coupling law taken from from https://icub-tech-iit.github.io/documentation/hands/hands_mk5_coupling
     */
    auto params = mFingerParameters.at(finger_name);
    double q1_rad = q1 * M_PI / 180.0;
    double q1off_rad = params.q1off * M_PI / 180.0;
    double q2bias_rad = params.q2bias * M_PI / 180.0;

    double P1x_q1 = params.d * cos(q1_rad + q1off_rad);
    double P1y_q1 = params.d * sin(q1_rad + q1off_rad);

    double h_sq = std::pow(P1x_q1 - params.L0x, 2) + std::pow(P1y_q1 - params.L0y, 2);
    double h = std::sqrt(h_sq);
    double l_sq = std::pow(params.l, 2);
    double k_sq = std::pow(params.k, 2);

    double q2 = atan2(P1y_q1 - params.L0y, P1x_q1 - params.L0x) + \
        acos((h_sq + l_sq - k_sq) / (2.0 * params.l * h)) + \
        -q2bias_rad - M_PI;

    // The value of q1 is subtracted from the result as the formula provides
    // the absolute angle of the coupled distal joint with respect to the palm.
    return q2 * 180.0 / M_PI - q1;
}

double HandMk5CouplingHandler::evaluateCoupledJointJacobian(const double& q1, const std::string& finger_name)
{
    /**
     * Coupling law jacobian taken from from https://icub-tech-iit.github.io/documentation/hands/hands_mk5_coupling
     */
    auto params = mFingerParameters.at(finger_name);
    double q1_rad = q1 * M_PI / 180.0;
    double q1off_rad = params.q1off * M_PI / 180.0;

    double P1x_q1 = params.d * cos(q1_rad + q1off_rad);
    double P1y_q1 = params.d * sin(q1_rad + q1off_rad);

    double h_sq = std::pow(P1x_q1 - params.L0x, 2) + std::pow(P1y_q1 - params.L0y, 2);
    double h = std::sqrt(h_sq);
    double l_sq = std::pow(params.l, 2);
    double k_sq = std::pow(params.k, 2);

    double dq2_dq1_11 = 1;
    double dq2_dq1_21 = 2 - (std::pow(params.d, 2) - std::pow(params.b, 2)) / (std::pow(params.d, 2) - (params.L0x * P1x_q1 + params.L0y * P1y_q1));
    double dq2_dq1_12 = (params.L0x * P1y_q1 - params.L0y * P1x_q1) * (l_sq - k_sq - h_sq);
    double dq2_dq1_22 = 2 * params.l * h * h_sq * std::sqrt(1 - std::pow((l_sq - k_sq + h_sq) / (2 * params.l * h), 2));
    double dq2_dq1 = dq2_dq1_11 / dq2_dq1_21 + dq2_dq1_12 / dq2_dq1_22;

    // The value of 1 is subtracted from the result as evaluateCoupledJointJacobian provides
    // the jacobian of the absolute angle of the coupled distal joint.
    return dq2_dq1 - 1;
}
