/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "ControlBoardDriver.h"
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/transport/Publisher.hh>
#include <yarp/os/LogStream.h>


using namespace yarp::dev;

bool GazeboYarpControlBoardDriver::getInteractionMode(int j, yarp::dev::InteractionModeEnum* mode)
{
    *mode = static_cast<yarp::dev::InteractionModeEnum>(m_interactionMode[j]);
    return true;
}

bool GazeboYarpControlBoardDriver::getInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    if (!modes) return false;
    bool ret = true;
    for(int i = 0; i < n_joints; ++i)
        ret = ret && getInteractionMode(joints[i], &modes[i]);
    return ret;
}

bool GazeboYarpControlBoardDriver::getInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    if (!modes) return false;
    for (size_t j = 0; j < m_numberOfJoints; ++j) {
        modes[j] = static_cast<yarp::dev::InteractionModeEnum>(m_interactionMode[j]);
    }
    return true;
}

bool GazeboYarpControlBoardDriver::changeInteractionMode(int j, yarp::dev::InteractionModeEnum mode)
{
    resetAllPidsForJointAtIndex(j);
    m_interactionMode[j] = static_cast<int>(mode);

    //the following code (copy and pasted from changeControlMode) is used to reset control references / trajectory generator to the current position etc.
    switch (m_controlMode[j])
    {
        case VOCAB_CM_POSITION :
            m_jntReferencePositions[j] = m_positions[j];
            m_trajectoryGenerationReferencePosition[j] = m_positions[j];
            m_trajectory_generator[j]->setLimits(m_jointPosLimits[j].min,m_jointPosLimits[j].max);
            m_trajectory_generator[j]->initTrajectory(m_positions[j],m_trajectoryGenerationReferencePosition[j],m_trajectoryGenerationReferenceSpeed[j]);
            break;
        case VOCAB_CM_POSITION_DIRECT :
            m_jntReferencePositions[j] = m_positions[j];
            m_trajectoryGenerationReferencePosition[j] = m_positions[j];
            break;
        case VOCAB_CM_VELOCITY :
            m_jntReferenceVelocities[j] = 0.0;
            break;
        case VOCAB_CM_MIXED:
            m_jntReferencePositions[j] = m_positions[j];
            m_trajectoryGenerationReferencePosition[j] = m_positions[j];
            m_jntReferenceVelocities[j] = 0.0;
            m_trajectory_generator[j]->setLimits(m_jointPosLimits[j].min,m_jointPosLimits[j].max);
            m_trajectory_generator[j]->initTrajectory(m_positions[j],m_trajectoryGenerationReferencePosition[j],m_trajectoryGenerationReferenceSpeed[j]);
            break;
        case VOCAB_CM_TORQUE :
        case VOCAB_CM_PWM :
        case VOCAB_CM_CURRENT :
            m_jntReferenceTorques[j] = m_torques[j];
            break;
        case VOCAB_CM_IDLE:
            m_jntReferenceTorques[j] = 0.0;
            break;
        default :
            break;
    }

    return true;
}

bool GazeboYarpControlBoardDriver::setInteractionMode(int j, yarp::dev::InteractionModeEnum mode)
{
    if (j < 0 || static_cast<size_t>(j) >= m_numberOfJoints) return false;


    for (size_t cpl_i=0; cpl_i < m_coupling_handler.size(); cpl_i++)
    {
        if (m_coupling_handler[cpl_i] && m_coupling_handler[cpl_i]->checkJointIsCoupled(j))
        {
            yarp::sig::VectorOf<int> coupling_vector = m_coupling_handler[cpl_i]->getCoupledJoints();
            for (size_t coupled_j=0; coupled_j < coupling_vector.size(); coupled_j++)
            {
                changeInteractionMode(coupling_vector[coupled_j], mode);
            }
            return true;
        }
    }
    changeInteractionMode(j,mode);

    return true;
}

bool GazeboYarpControlBoardDriver::setInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    bool ret = true;
    for (int i = 0; i < n_joints; i++)
        ret = ret && setInteractionMode(joints[i], modes[i]);
    return ret;
}

bool GazeboYarpControlBoardDriver::setInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    bool ret = true;
    for (size_t i = 0; i < m_numberOfJoints; ++i) {
        ret = ret && setInteractionMode(i, modes[i]);
    }
    return ret;
}
