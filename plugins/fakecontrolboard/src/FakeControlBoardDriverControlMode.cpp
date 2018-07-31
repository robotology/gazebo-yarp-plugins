/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "FakeControlBoardDriver.h"

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/transport/Publisher.hh>

#include <yarp/os/Vocab.h>
#include <yarp/os/LogStream.h>

using namespace yarp::dev;

bool GazeboYarpFakeControlBoardDriver::getControlModes(const int n_joint, const int *joints, int *modes)
{
    bool ret = true;
    for (int i = 0; i < n_joint; i++)
        ret = ret && getControlMode(joints[i], &modes[i]);
    return ret;
}

bool GazeboYarpFakeControlBoardDriver::getControlModes(int *modes)
{
    if (!modes) return false;
    for(unsigned int j = 0; j < m_numberOfJoints; ++j) {
        modes[j] = m_controlMode[j];
    }
    return true;
}

bool GazeboYarpFakeControlBoardDriver::getControlMode(int j, int *mode)
{
    if (!mode || j < 0 || j >= (int)m_controlMode.size())
        return false;
    *mode = m_controlMode[j];
    return true;
}

bool GazeboYarpFakeControlBoardDriver::setControlModes(const int n_joint, const int *joints, int *modes)
{
    return getTrueIfArgumentIsZero(n_joint);
}

bool GazeboYarpFakeControlBoardDriver::setControlMode(const int, const int)
{
    return false;
}
bool GazeboYarpFakeControlBoardDriver::setControlModes(int *)
{
    return false;
}
