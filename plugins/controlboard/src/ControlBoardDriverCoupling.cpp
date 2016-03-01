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

bool EyesCouplingHandler::decouplePos (yarp::sig::Vector& current_pos)
{
  if (m_coupledJoints.size()!=2) return false;
  double temp = current_pos[m_coupledJoints[0]];
  current_pos[m_coupledJoints[0]] = temp + current_pos[m_coupledJoints[1]];
  current_pos[m_coupledJoints[1]] = temp - current_pos[m_coupledJoints[1]];
  return true;
}

bool EyesCouplingHandler::decoupleVel (yarp::sig::Vector& current_vel)
{
  if (m_coupledJoints.size()!=2) return false;
  double temp = current_vel[m_coupledJoints[0]];
  current_vel[m_coupledJoints[0]] = temp + current_vel[m_coupledJoints[1]];
  current_vel[m_coupledJoints[1]] = temp - current_vel[m_coupledJoints[1]];
  return true;
}

bool EyesCouplingHandler::decoupleAcc (yarp::sig::Vector& current_acc)
{
  if (m_coupledJoints.size()!=2) return false;
  double temp = current_acc[m_coupledJoints[0]];
  current_acc[m_coupledJoints[0]] = temp + current_acc[m_coupledJoints[1]];
  current_acc[m_coupledJoints[1]] = temp - current_acc[m_coupledJoints[1]];
  return true;
}

bool EyesCouplingHandler::decoupleTrq (yarp::sig::Vector& current_trq)
{
  if (m_coupledJoints.size()!=2) return false;
  return false;
}

yarp::sig::Vector EyesCouplingHandler::decoupleRefPos (yarp::sig::Vector& pos_ref)
{
  yarp::sig::Vector out = pos_ref;
  if (m_coupledJoints.size()!=2) {yError() << "Invalild coupling vector"; return out;}
  out[m_coupledJoints[0]] = (pos_ref[m_coupledJoints[0]] + pos_ref[m_coupledJoints[1]])/2;
  out[m_coupledJoints[1]] = (pos_ref[m_coupledJoints[0]] - pos_ref[m_coupledJoints[1]])/2;
  return out;
}

yarp::sig::Vector EyesCouplingHandler::decoupleRefVel (yarp::sig::Vector& vel_ref)
{
  yarp::sig::Vector out = vel_ref;
  if (m_coupledJoints.size()!=2) {yError() << "Invalild coupling vector"; return out;}
  out[m_coupledJoints[0]] = (vel_ref[m_coupledJoints[0]] + vel_ref[m_coupledJoints[1]])/2;
  out[m_coupledJoints[1]] = (vel_ref[m_coupledJoints[0]] - vel_ref[m_coupledJoints[1]])/2;
  return out;
}

yarp::sig::Vector EyesCouplingHandler::decoupleRefTrq (yarp::sig::Vector& trq_ref)
{
  yarp::sig::Vector out =trq_ref;
  if (m_coupledJoints.size()!=2) {yError() << "Invalild coupling vector"; return out;}
  out[m_coupledJoints[0]] = (trq_ref[m_coupledJoints[0]] + trq_ref[m_coupledJoints[1]])/2;
  out[m_coupledJoints[1]] = (trq_ref[m_coupledJoints[0]] - trq_ref[m_coupledJoints[1]])/2;
  return out;
}
