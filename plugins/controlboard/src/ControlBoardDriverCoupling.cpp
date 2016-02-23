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

bool EyesCouplingHandler::decouplePos (yarp::sig::Vector& current_pos)
{
}

bool EyesCouplingHandler::decoupleVel (yarp::sig::Vector& current_vel)
{
}

bool EyesCouplingHandler::decoupleAcc (yarp::sig::Vector& current_acc)
{
}

bool EyesCouplingHandler::decoupleTrq (yarp::sig::Vector& current_trq)
{
}