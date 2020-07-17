/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: Marco Randazzo <marco.randazzo@iit.it>
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "ControlBoardDriver.h"
#include <GazeboYarpPlugins/common.h>

#include <ControlBoardDriverCoupling.h>
#include <cstdio>
#include <cmath>
#include <gazebo/physics/Model.hh>

#include <yarp/os/LogStream.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

//------------------------------------------------------------------------------------------------------------------
// BaseCouplingHandler
//------------------------------------------------------------------------------------------------------------------

BaseCouplingHandler::BaseCouplingHandler(gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names)
{
    m_robot = model;
    m_coupledJoints=coupled_joints;
    m_coupledJointNames=coupled_joint_names;
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

//------------------------------------------------------------------------------------------------------------------
// EyesCouplingHandler
//------------------------------------------------------------------------------------------------------------------

EyesCouplingHandler::EyesCouplingHandler(gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names)
: BaseCouplingHandler(model, coupled_joints,coupled_joint_names)
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
    if (m_coupledJoints.size() != m_couplingSize) {yError() << "EyesCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = (pos_ref[m_coupledJoints[0]] + pos_ref[m_coupledJoints[1]])/2;
    out[m_coupledJoints[1]] = (pos_ref[m_coupledJoints[0]] - pos_ref[m_coupledJoints[1]])/2;
    return out;
}

yarp::sig::Vector EyesCouplingHandler::decoupleRefVel (yarp::sig::Vector& vel_ref)
{
    yarp::sig::Vector out = vel_ref;
    if (m_coupledJoints.size() != m_couplingSize) {yError() << "EyesCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = (vel_ref[m_coupledJoints[0]] + vel_ref[m_coupledJoints[1]])/2;
    out[m_coupledJoints[1]] = (vel_ref[m_coupledJoints[0]] - vel_ref[m_coupledJoints[1]])/2;
    return out;
}

yarp::sig::Vector EyesCouplingHandler::decoupleRefTrq (yarp::sig::Vector& trq_ref)
{
    yarp::sig::Vector out = trq_ref;
    if (m_coupledJoints.size() != m_couplingSize) {yError() << "EyesCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = (trq_ref[m_coupledJoints[0]] + trq_ref[m_coupledJoints[1]])/2;
    out[m_coupledJoints[1]] = (trq_ref[m_coupledJoints[0]] - trq_ref[m_coupledJoints[1]])/2;
    return out;
}

//------------------------------------------------------------------------------------------------------------------
// ThumbCouplingHandler
//------------------------------------------------------------------------------------------------------------------

ThumbCouplingHandler::ThumbCouplingHandler(gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names)
: BaseCouplingHandler(model, coupled_joints,coupled_joint_names)
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
    if (m_coupledJoints.size()!=m_couplingSize) {yError() << "ThumbCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = pos_ref[m_coupledJoints[0]];
    out[m_coupledJoints[1]] = pos_ref[m_coupledJoints[1]];
    out[m_coupledJoints[2]] = pos_ref[m_coupledJoints[2]]/2;
    out[m_coupledJoints[3]] = pos_ref[m_coupledJoints[2]]/2;
    return out;
}

yarp::sig::Vector ThumbCouplingHandler::decoupleRefVel (yarp::sig::Vector& vel_ref)
{
    yarp::sig::Vector out = vel_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yError() << "ThumbCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = vel_ref[m_coupledJoints[0]];
    out[m_coupledJoints[1]] = vel_ref[m_coupledJoints[1]];
    out[m_coupledJoints[2]] = vel_ref[m_coupledJoints[2]]/2;
    out[m_coupledJoints[3]] = vel_ref[m_coupledJoints[2]]/2;
    return out;
}

yarp::sig::Vector ThumbCouplingHandler::decoupleRefTrq (yarp::sig::Vector& trq_ref)
{
    yarp::sig::Vector out =trq_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yError() << "ThumbCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = trq_ref[m_coupledJoints[0]];
    out[m_coupledJoints[1]] = trq_ref[m_coupledJoints[1]];
    out[m_coupledJoints[2]] = trq_ref[m_coupledJoints[2]]/2;
    out[m_coupledJoints[3]] = trq_ref[m_coupledJoints[2]]/2;
    return out;
}

//------------------------------------------------------------------------------------------------------------------
// IndexCouplingHandler
//------------------------------------------------------------------------------------------------------------------

IndexCouplingHandler::IndexCouplingHandler(gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names)
: BaseCouplingHandler(model, coupled_joints,coupled_joint_names)
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
    if (m_coupledJoints.size()!=m_couplingSize) {yError() << "IndexCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = pos_ref[m_coupledJoints[0]];
    out[m_coupledJoints[1]] = pos_ref[m_coupledJoints[1]]/2;
    out[m_coupledJoints[2]] = pos_ref[m_coupledJoints[1]]/2;
    return out;
}

yarp::sig::Vector IndexCouplingHandler::decoupleRefVel (yarp::sig::Vector& vel_ref)
{
    yarp::sig::Vector out = vel_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yError() << "IndexCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = vel_ref[m_coupledJoints[0]];
    out[m_coupledJoints[1]] = vel_ref[m_coupledJoints[1]]/2;
    out[m_coupledJoints[2]] = vel_ref[m_coupledJoints[1]]/2;
    return out;
}

yarp::sig::Vector IndexCouplingHandler::decoupleRefTrq (yarp::sig::Vector& trq_ref)
{
    yarp::sig::Vector out =trq_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yError() << "IndexCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = trq_ref[m_coupledJoints[0]];
    out[m_coupledJoints[1]] = trq_ref[m_coupledJoints[1]]/2;
    out[m_coupledJoints[2]] = trq_ref[m_coupledJoints[1]]/2;
    return out;
}

//------------------------------------------------------------------------------------------------------------------
// MiddleCouplingHandler
//------------------------------------------------------------------------------------------------------------------

MiddleCouplingHandler::MiddleCouplingHandler(gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names)
: BaseCouplingHandler(model, coupled_joints,coupled_joint_names)
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
    if (m_coupledJoints.size()!=m_couplingSize) {yError() << "MiddleCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = pos_ref[m_coupledJoints[0]];
    out[m_coupledJoints[1]] = pos_ref[m_coupledJoints[1]]/2;
    out[m_coupledJoints[2]] = pos_ref[m_coupledJoints[1]]/2;
    return out;
}

yarp::sig::Vector MiddleCouplingHandler::decoupleRefVel (yarp::sig::Vector& vel_ref)
{
    yarp::sig::Vector out = vel_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yError() << "MiddleCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = vel_ref[m_coupledJoints[0]];
    out[m_coupledJoints[1]] = vel_ref[m_coupledJoints[1]]/2;
    out[m_coupledJoints[2]] = vel_ref[m_coupledJoints[1]]/2;
    return out;
}

yarp::sig::Vector MiddleCouplingHandler::decoupleRefTrq (yarp::sig::Vector& trq_ref)
{
    yarp::sig::Vector out =trq_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yError() << "MiddleCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = trq_ref[m_coupledJoints[0]];
    out[m_coupledJoints[1]] = trq_ref[m_coupledJoints[1]]/2;
    out[m_coupledJoints[2]] = trq_ref[m_coupledJoints[1]]/2;
    return out;
}

//------------------------------------------------------------------------------------------------------------------
// PinkyCouplingHandler
//------------------------------------------------------------------------------------------------------------------

PinkyCouplingHandler::PinkyCouplingHandler(gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names)
: BaseCouplingHandler(model, coupled_joints,coupled_joint_names)
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
    if (m_coupledJoints.size()!=m_couplingSize) {yError() << "PinkyCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = pos_ref[m_coupledJoints[0]]/3;
    out[m_coupledJoints[1]] = pos_ref[m_coupledJoints[0]]/3;
    out[m_coupledJoints[2]] = pos_ref[m_coupledJoints[0]]/3;
    out[m_coupledJoints[3]] = pos_ref[m_coupledJoints[0]]/3;
    out[m_coupledJoints[4]] = pos_ref[m_coupledJoints[0]]/3;
    out[m_coupledJoints[5]] = pos_ref[m_coupledJoints[0]]/3;
    return out;
}

yarp::sig::Vector PinkyCouplingHandler::decoupleRefVel (yarp::sig::Vector& vel_ref)
{
    yarp::sig::Vector out = vel_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yError() << "PinkyCouplingHandler: Invalid coupling vector"; return out;}
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
    if (m_coupledJoints.size()!=m_couplingSize) {yError() << "PinkyCouplingHandler: Invalid coupling vector"; return out;}
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

FingersAbductionCouplingHandler::FingersAbductionCouplingHandler(gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names)
: BaseCouplingHandler(model, coupled_joints,coupled_joint_names)
{
    m_couplingSize = 4;
}

bool FingersAbductionCouplingHandler::decouplePos (yarp::sig::Vector& current_pos)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    current_pos[m_coupledJoints[0]] = current_pos[m_coupledJoints[3]];
    return true;
}

bool FingersAbductionCouplingHandler::decoupleVel (yarp::sig::Vector& current_vel)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    current_vel[m_coupledJoints[0]] = current_vel[m_coupledJoints[3]];
    return true;
}

bool FingersAbductionCouplingHandler::decoupleAcc (yarp::sig::Vector& current_acc)
{

    if (m_coupledJoints.size()!=m_couplingSize) return false;
    current_acc[m_coupledJoints[0]] = current_acc[m_coupledJoints[3]];
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
    if (m_coupledJoints.size()!=m_couplingSize) {yError() << "FingersAbductionCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = -pos_ref[m_coupledJoints[0]]/2;
    out[m_coupledJoints[1]] = 0.0;
    out[m_coupledJoints[2]] = pos_ref[m_coupledJoints[0]]/2;
    out[m_coupledJoints[3]] = pos_ref[m_coupledJoints[0]];
    return out;
}

yarp::sig::Vector FingersAbductionCouplingHandler::decoupleRefVel (yarp::sig::Vector& vel_ref)
{
    yarp::sig::Vector out = vel_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yError() << "FingersAbductionCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = -vel_ref[m_coupledJoints[0]]/2;
    out[m_coupledJoints[1]] = 0.0;
    out[m_coupledJoints[2]] = vel_ref[m_coupledJoints[0]]/2;
    out[m_coupledJoints[3]] = vel_ref[m_coupledJoints[0]];
    return out;
}

yarp::sig::Vector FingersAbductionCouplingHandler::decoupleRefTrq (yarp::sig::Vector& trq_ref)
{
    yarp::sig::Vector out =trq_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yError() << "FingersAbductionCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = -trq_ref[m_coupledJoints[0]]/2;
    out[m_coupledJoints[1]] = 0.0;
    out[m_coupledJoints[2]] = trq_ref[m_coupledJoints[0]]/2;
    out[m_coupledJoints[3]] = trq_ref[m_coupledJoints[0]];
    return out;
}

//------------------------------------------------------------------------------------------------------------------
// CerHandCouplingHandler
//------------------------------------------------------------------------------------------------------------------

CerHandCouplingHandler::CerHandCouplingHandler(gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names)
: BaseCouplingHandler(model, coupled_joints,coupled_joint_names)
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
    if (m_coupledJoints.size()!=m_couplingSize) {yError() << "CerHandCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = pos_ref[m_coupledJoints[0]]/2;
    out[m_coupledJoints[1]] = pos_ref[m_coupledJoints[0]]/2;
    out[m_coupledJoints[2]] = pos_ref[m_coupledJoints[1]]/2;
    out[m_coupledJoints[3]] = pos_ref[m_coupledJoints[1]]/2;
    return out;
}

yarp::sig::Vector CerHandCouplingHandler::decoupleRefVel (yarp::sig::Vector& vel_ref)
{
    yarp::sig::Vector out = vel_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yError() << "CerHandCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = vel_ref[m_coupledJoints[0]]/2;
    out[m_coupledJoints[1]] = vel_ref[m_coupledJoints[0]]/2;
    out[m_coupledJoints[2]] = vel_ref[m_coupledJoints[1]]/2;
    out[m_coupledJoints[3]] = vel_ref[m_coupledJoints[1]]/2;
    return out;
}

yarp::sig::Vector CerHandCouplingHandler::decoupleRefTrq (yarp::sig::Vector& trq_ref)
{
    yarp::sig::Vector out =trq_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yError() << "CerHandCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = trq_ref[m_coupledJoints[0]]/2;
    out[m_coupledJoints[1]] = trq_ref[m_coupledJoints[0]]/2;
    out[m_coupledJoints[2]] = trq_ref[m_coupledJoints[1]]/2;
    out[m_coupledJoints[3]] = trq_ref[m_coupledJoints[1]]/2;
    return out;
}

//------------------------------------------------------------------------------------------------------------------
// HandMk3CouplingHandler
//------------------------------------------------------------------------------------------------------------------

HandMk3CouplingHandler::HandMk3CouplingHandler(gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names)
: BaseCouplingHandler(model, coupled_joints,coupled_joint_names), LUTSIZE(4096)
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
            
            double dindex = q2*10.0;
            
            int iindex = int(dindex);
            
            double w = dindex - double(iindex);
            
            index_lut[iindex] += (1.0 - w)*q1;
            num[iindex] += (1.0 - w);
            
            index_lut[iindex + 1] += w*q1;
            num[iindex + 1] += w;
        }
        
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
    double w = dindex - double(iindex);
    return lut[iindex]*(1.0 - w) + lut[iindex + 1]*w;
}

yarp::sig::Vector HandMk3CouplingHandler::decoupleRefPos (yarp::sig::Vector& pos_ref)
{
    yarp::sig::Vector out = pos_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yError() << "HandMk3CouplingHandler: Invalid coupling vector"; return out;}
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

yarp::sig::Vector HandMk3CouplingHandler::decoupleRefVel (yarp::sig::Vector& vel_ref)
{
    yarp::sig::Vector out = vel_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yError() << "HandMk3CouplingHandler: Invalid coupling vector"; return out;}
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
    if (m_coupledJoints.size()!=m_couplingSize) {yError() << "HandMk3CouplingHandler: Invalid coupling vector"; return out;}
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
