/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "ControlBoardDriver.h"
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/transport/Publisher.hh>


using namespace yarp::dev;

bool GazeboYarpControlBoardDriver::getInteractionMode(int j, yarp::dev::InteractionModeEnum* mode)
{
    *mode = (yarp::dev::InteractionModeEnum) m_interactionMode[j];
    return true;
}

bool GazeboYarpControlBoardDriver::getInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    if (!modes) return false;
    bool ret = true;
    for(int i=0; i<n_joints; i++)
        ret = ret && getInteractionMode(joints[i], &modes[i]);
    return ret;
}

bool GazeboYarpControlBoardDriver::getInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    if (!modes) return false;
    for(unsigned int j = 0; j < m_numberOfJoints; ++j) {
        modes[j] = (yarp::dev::InteractionModeEnum) m_controlMode[j];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::setInteractionMode(int j, yarp::dev::InteractionModeEnum mode)
{
    if (j < 0 || j >= (int)m_numberOfJoints) return false;
    prepareResetJointMsg(j);
    m_interactionMode[j] = (int) mode;
    return true;
}

bool GazeboYarpControlBoardDriver::setInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    bool ret = true;
    for(int i=0; i<n_joints; i++)
        ret = ret && setInteractionMode(joints[i], modes[i]);
    return ret;
}

bool GazeboYarpControlBoardDriver::setInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    bool ret = true;
    for(int i=0; i<(int)m_numberOfJoints; i++)
        ret = ret && setControlMode(i, modes[i]);
    return ret;
}
