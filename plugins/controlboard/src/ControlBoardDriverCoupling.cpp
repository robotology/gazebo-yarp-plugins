/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: Marco Randazzo <marco.randazzo@iit.it>
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "ControlBoardDriver.h"
#include <GazeboYarpPlugins/common.h>

#include <ControlBoardDriverCoupling.h>
#include <cstdio>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/math/Angle.hh>

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
    for (size_t i=0; i<m_coupledJoints.size(); i++)
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
    for (size_t i =0; i<m_coupledJoints.size(); i++)
    {
        if (m_coupledJoints[i]==joint) c_joint = i;
    }
               
    if (c_joint>=0 && c_joint < m_coupledJoints.size())
    {
        return m_coupledJointNames[c_joint];
    }
    else
    {
        return std::string("invalid");
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
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    double temp = current_pos[m_coupledJoints[0]];
    current_pos[m_coupledJoints[0]] = temp + current_pos[m_coupledJoints[1]];
    current_pos[m_coupledJoints[1]] = temp - current_pos[m_coupledJoints[1]];
    return true;
}

bool EyesCouplingHandler::decoupleVel (yarp::sig::Vector& current_vel)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    double temp = current_vel[m_coupledJoints[0]];
    current_vel[m_coupledJoints[0]] = temp + current_vel[m_coupledJoints[1]];
    current_vel[m_coupledJoints[1]] = temp - current_vel[m_coupledJoints[1]];
    return true;
}

bool EyesCouplingHandler::decoupleAcc (yarp::sig::Vector& current_acc)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    double temp = current_acc[m_coupledJoints[0]];
    current_acc[m_coupledJoints[0]] = temp + current_acc[m_coupledJoints[1]];
    current_acc[m_coupledJoints[1]] = temp - current_acc[m_coupledJoints[1]];
    return true;
}

bool EyesCouplingHandler::decoupleTrq (yarp::sig::Vector& current_trq)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    return false;
}

yarp::sig::Vector EyesCouplingHandler::decoupleRefPos (yarp::sig::Vector& pos_ref)
{
    yarp::sig::Vector out = pos_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yError() << "EyesCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = (pos_ref[m_coupledJoints[0]] + pos_ref[m_coupledJoints[1]])/2;
    out[m_coupledJoints[1]] = (pos_ref[m_coupledJoints[0]] - pos_ref[m_coupledJoints[1]])/2;
    return out;
}

yarp::sig::Vector EyesCouplingHandler::decoupleRefVel (yarp::sig::Vector& vel_ref)
{
    yarp::sig::Vector out = vel_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yError() << "EyesCouplingHandler: Invalid coupling vector"; return out;}
    out[m_coupledJoints[0]] = (vel_ref[m_coupledJoints[0]] + vel_ref[m_coupledJoints[1]])/2;
    out[m_coupledJoints[1]] = (vel_ref[m_coupledJoints[0]] - vel_ref[m_coupledJoints[1]])/2;
    return out;
}

yarp::sig::Vector EyesCouplingHandler::decoupleRefTrq (yarp::sig::Vector& trq_ref)
{
    yarp::sig::Vector out =trq_ref;
    if (m_coupledJoints.size()!=m_couplingSize) {yError() << "EyesCouplingHandler: Invalid coupling vector"; return out;}
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
    if (m_coupledJoints.size()!=m_couplingSize) return false;
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

bool MiddleCouplingHandler::decouplePos (yarp::sig::Vector& current_pos)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    double temp = current_pos[m_coupledJoints[0]];
    current_pos[m_coupledJoints[1]] = current_pos[m_coupledJoints[1]] + current_pos[m_coupledJoints[2]];
    return true;
}

bool MiddleCouplingHandler::decoupleVel (yarp::sig::Vector& current_vel)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    double temp = current_vel[m_coupledJoints[0]];
    current_vel[m_coupledJoints[1]] = current_vel[m_coupledJoints[1]] + current_vel[m_coupledJoints[2]];
    return true;
}

bool MiddleCouplingHandler::decoupleAcc (yarp::sig::Vector& current_acc)
{
    if (m_coupledJoints.size()!=m_couplingSize) return false;
    double temp = current_acc[m_coupledJoints[0]];
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