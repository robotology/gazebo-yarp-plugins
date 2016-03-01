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
  double temp = current_pos[0];
  current_pos[0] = temp + current_pos[1];
  current_pos[1] = temp - current_pos[1];
  return true;
}

bool EyesCouplingHandler::decoupleVel (yarp::sig::Vector& current_vel)
{
  double temp = current_vel[0];
  current_vel[0] = temp + current_vel[1];
  current_vel[1] = temp - current_vel[1];
  return true;
}

bool EyesCouplingHandler::decoupleAcc (yarp::sig::Vector& current_acc)
{
  double temp = current_acc[0];
  current_acc[0] = temp + current_acc[1];
  current_acc[1] = temp - current_acc[1];
  return true;
}

bool EyesCouplingHandler::decoupleTrq (yarp::sig::Vector& current_trq)
{
  return false;
}

yarp::sig::Vector EyesCouplingHandler::decoupleRefPos (yarp::sig::Vector& pos_ref)
{
  yarp::sig::Vector out (2,0.0);
  out[0] = (pos_ref[0] + pos_ref[1])/2;
  out[1] = (pos_ref[0] - pos_ref[1])/2;
  return out;
}

yarp::sig::Vector EyesCouplingHandler::decoupleRefVel (yarp::sig::Vector& vel_ref)
{
  yarp::sig::Vector out (2,0.0);
  out[0] = (vel_ref[0] + vel_ref[1])/2;
  out[1] = (vel_ref[0] - vel_ref[1])/2;
  return out;
}

yarp::sig::Vector EyesCouplingHandler::decoupleRefTrq (yarp::sig::Vector& trq_ref)
{
  yarp::sig::Vector out (2,0.0);
  out[0] = (trq_ref[0] + trq_ref[1])/2;
  out[1] = (trq_ref[0] - trq_ref[1])/2;
  return out;
}
