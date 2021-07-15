/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "ControlBoardDriver.h"

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/transport/Publisher.hh>

#include <yarp/os/Vocab.h>
#include <yarp/os/LogStream.h>

using namespace yarp::dev;

void GazeboYarpControlBoardDriver::resetAllPidsForJointAtIndex(int j)
{
    m_pids[VOCAB_PIDTYPE_POSITION][j].Reset();
    m_pids[VOCAB_PIDTYPE_VELOCITY][j].Reset();
}

bool GazeboYarpControlBoardDriver::getControlMode(int j, int *mode)
{
    if (!mode || j < 0 || static_cast<size_t>(j) >= m_numberOfJoints)
        return false;
    *mode = m_controlMode[j];
    return true;
}

bool GazeboYarpControlBoardDriver::getControlModes(int *modes) //NOT TESTED
{
    if (!modes) return false;
    for(size_t j = 0; j < m_numberOfJoints; ++j) {
        modes[j] = m_controlMode[j];
    }
    return true;
}


bool GazeboYarpControlBoardDriver::getControlModes(const int n_joint, const int *joints, int *modes)
{
    bool ret = true;
    for (int i = 0; i < n_joint; i++) {
        ret = ret && getControlMode(joints[i], &modes[i]);
    }
    return ret;
}

bool GazeboYarpControlBoardDriver::setControlMode(const int j, const int mode)
{
    if (j < 0 || static_cast<size_t>(j) >= m_numberOfJoints) return false;

    // Only accept supported control modes
    // The only not supported control mode is
    // (for now) VOCAB_CM_MIXED
    if (!(mode == VOCAB_CM_POSITION
          || mode == VOCAB_CM_POSITION_DIRECT
          || mode == VOCAB_CM_VELOCITY
          || mode == VOCAB_CM_TORQUE
          || mode == VOCAB_CM_MIXED
          || mode == VOCAB_CM_PWM
          || mode == VOCAB_CM_CURRENT
          || mode == VOCAB_CM_IDLE
          || mode == VOCAB_CM_FORCE_IDLE)) {
        yWarning() << "request control mode "
        << yarp::os::Vocab32::decode(mode) << " that is not supported by "
        << " gazebo_yarp_controlboard plugin.";
        return false;
    }

    for (size_t cpl_i = 0; cpl_i < m_coupling_handler.size(); ++cpl_i)
    {
        if (m_coupling_handler[cpl_i] && m_coupling_handler[cpl_i]->checkJointIsCoupled(j))
        {
            yarp::sig::VectorOf<int> coupling_vector = m_coupling_handler[cpl_i]->getCoupledJoints();
            for (size_t coupled_j = 0; coupled_j < coupling_vector.size(); ++coupled_j)
            {
                changeControlMode(coupling_vector[coupled_j], mode);
            }
            return true;
        }
    }
    changeControlMode(j,mode);
    return true;
}

bool GazeboYarpControlBoardDriver::changeControlMode(const int j, const int mode)
{
    int desired_mode = mode;

    //if joint is in hw fault, only a force idle command can recover it
    if (m_controlMode[j] == VOCAB_CM_HW_FAULT && mode != VOCAB_CM_FORCE_IDLE)
    {
        return true;
    }

    if (mode == VOCAB_CM_FORCE_IDLE)
    {
        //clean the fault status (missing) and set control mode to idle
        desired_mode = VOCAB_CM_IDLE;
    }

    // If the joint is already in the selected control mode
    // don't perform switch specific actions
    if (m_controlMode[j] == desired_mode) return true;

    resetAllPidsForJointAtIndex(j);
    m_controlMode[j] = desired_mode;

    // get limits for trajectory generation
    double limit_min, limit_max;
    getUserDOFLimit(j, limit_min, limit_max);

    // mode specific switching actions
    switch (desired_mode) {
        case VOCAB_CM_POSITION :
            m_jntReferencePositions[j] = m_positions[j];
            m_trajectoryGenerationReferencePosition[j] = m_positions[j];
            m_trajectory_generator[j]->setLimits(limit_min, limit_max);
            m_trajectory_generator[j]->initTrajectory(m_positions[j],
                                                      m_trajectoryGenerationReferencePosition[j],
                                                      m_trajectoryGenerationReferenceSpeed[j],
                                                      m_trajectoryGenerationReferenceAcceleration[j]);
            break;
        case VOCAB_CM_POSITION_DIRECT :
            m_jntReferencePositions[j] = m_positions[j];
            m_trajectoryGenerationReferencePosition[j] = m_positions[j];
            break;
        case VOCAB_CM_VELOCITY :
            if (m_velocity_control_type == IntegratorAndPositionPID) {
                m_jntReferencePositions[j] = m_positions[j];
            }
            m_jntReferenceVelocities[j] = 0.0;
            m_speed_ramp_handler[j]->stop();
            break;
        case VOCAB_CM_MIXED:
            m_jntReferencePositions[j] = m_positions[j];
            m_trajectoryGenerationReferencePosition[j] = m_positions[j];
            m_jntReferenceVelocities[j] = 0.0;
            m_speed_ramp_handler[j]->stop();
            m_trajectory_generator[j]->setLimits(limit_min, limit_max);
            m_trajectory_generator[j]->initTrajectory(m_positions[j],
                                                      m_trajectoryGenerationReferencePosition[j],
                                                      m_trajectoryGenerationReferenceSpeed[j],
                                                      m_trajectoryGenerationReferenceAcceleration[j]);
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

bool GazeboYarpControlBoardDriver::setControlModes(const int n_joint, const int *joints, int *modes)
{

    bool ret = true;
    for (int i = 0; i < n_joint; i++) {
        ret = ret && setControlMode(joints[i], modes[i]);
    }
    return ret;
}

bool GazeboYarpControlBoardDriver::setControlModes(int *modes)
{
    bool ret = true;
    for (size_t i = 0; i < m_numberOfJoints; ++i) {
        ret = ret && setControlMode(i, modes[i]);
    }
    return ret;
}
