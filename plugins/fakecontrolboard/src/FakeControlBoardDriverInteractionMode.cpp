/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "FakeControlBoardDriver.h"
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/transport/Publisher.hh>
#include <yarp/os/LogStream.h>


using namespace yarp::dev;

bool GazeboYarpFakeControlBoardDriver::getInteractionMode(int j, yarp::dev::InteractionModeEnum* mode)
{
    *mode = (yarp::dev::InteractionModeEnum) m_interactionMode[j];
    return true;
}

bool GazeboYarpFakeControlBoardDriver::getInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    if (!modes) return false;
    bool ret = true;
    for(int i=0; i<n_joints; i++)
        ret = ret && getInteractionMode(joints[i], &modes[i]);
    return ret;
}

bool GazeboYarpFakeControlBoardDriver::getInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    if (!modes) return false;
    for(unsigned int j = 0; j < m_numberOfJoints; ++j) {
        modes[j] = (yarp::dev::InteractionModeEnum) m_interactionMode[j];
    }
    return true;
}

bool GazeboYarpFakeControlBoardDriver::changeInteractionMode(int, yarp::dev::InteractionModeEnum ) {return false;} 
bool GazeboYarpFakeControlBoardDriver::setInteractionMode(int, yarp::dev::InteractionModeEnum ) {return false;} 
bool GazeboYarpFakeControlBoardDriver::setInteractionModes(int, int *, yarp::dev::InteractionModeEnum*) {return false;} 
bool GazeboYarpFakeControlBoardDriver::setInteractionModes(yarp::dev::InteractionModeEnum* ) {return false;} 

