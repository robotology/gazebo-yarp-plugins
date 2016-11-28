/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "ControlBoardDriver.h"

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/transport/Publisher.hh>
#include <boost/iterator/iterator_concepts.hpp>

#include <yarp/os/Vocab.h>
#include <yarp/os/LogStream.h>

using namespace yarp::dev;

void GazeboYarpControlBoardDriver::prepareResetJointMsg(int j)
{
    gazebo::msgs::JointCmd j_cmd;
    j_cmd.set_reset(true);
    j_cmd.set_name(this->m_robot->GetJoint(m_jointNames[j])->GetScopedName());
    this->m_jointCommandPublisher->WaitForConnection();
    this->m_jointCommandPublisher->Publish(j_cmd);
}

bool GazeboYarpControlBoardDriver::setPositionMode(int j)
{
    return this->setControlMode(j, VOCAB_CM_POSITION);
}

bool GazeboYarpControlBoardDriver::setVelocityMode(int j)
{
    return this->setControlMode(j, VOCAB_CM_VELOCITY);
}

bool GazeboYarpControlBoardDriver::getControlMode(int j, int *mode)
{
    if (!mode || j < 0 || j >= (int)m_numberOfJoints)
        return false;
    *mode = m_controlMode[j];
    return true;
}

bool GazeboYarpControlBoardDriver::getControlModes(int *modes) //NOT TESTED
{
    if (!modes) return false;
    for(unsigned int j = 0; j < m_numberOfJoints; ++j) {
        modes[j] = m_controlMode[j];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::setTorqueMode(int j)
{
    return this->setControlMode(j, VOCAB_CM_TORQUE);
}

bool GazeboYarpControlBoardDriver::setImpedancePositionMode(int j)//NOT TESTED
{
    bool ret = true;
    ret = ret && this->setControlMode(j, VOCAB_CM_POSITION);
    ret = ret && this->setInteractionMode(j, VOCAB_IM_COMPLIANT);
    return ret;
}

bool GazeboYarpControlBoardDriver::setImpedanceVelocityMode(int) //NOT IMPLEMENTED
{
    return false;
}

bool GazeboYarpControlBoardDriver::setOpenLoopMode(int j) //NOT IMPLEMENTED
{
    return this->setControlMode(j, VOCAB_CM_OPENLOOP);
}

bool GazeboYarpControlBoardDriver::getControlModes(const int n_joint, const int *joints, int *modes)
{
    bool ret = true;
    for (int i = 0; i < n_joint; i++)
        ret = ret && getControlMode(joints[i], &modes[i]);
    return ret;
}

bool GazeboYarpControlBoardDriver::setControlMode(const int j, const int mode)
{
    if (j < 0 || j >= (int)m_numberOfJoints) return false;

    // Only accept supported control modes
    // The only not supported control mode is
    // (for now) VOCAB_CM_MIXED
    if (!(mode == VOCAB_CM_POSITION
          || mode == VOCAB_CM_POSITION_DIRECT
          || mode == VOCAB_CM_VELOCITY
          || mode == VOCAB_CM_TORQUE
          || mode == VOCAB_CM_MIXED
          || mode == VOCAB_CM_OPENLOOP
          || mode == VOCAB_CM_IDLE
          || mode == VOCAB_CM_FORCE_IDLE)) {
        yWarning() << "request control mode "
                  << yarp::os::Vocab::decode(mode) << " that is not supported by "
                  << " gazebo_yarp_controlboard plugin.";
        return false;
    }
    
    for (int cpl_i=0; cpl_i<(int)m_coupling_handler.size(); cpl_i++)
    {
      if (m_coupling_handler[cpl_i] && m_coupling_handler[cpl_i]->checkJointIsCoupled(j))
      {
        yarp::sig::VectorOf<int> coupling_vector = m_coupling_handler[cpl_i]->getCoupledJoints();
        for (int coupled_j=0; coupled_j<coupling_vector.size(); coupled_j++)
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

    prepareResetJointMsg(j);
    m_controlMode[j] = desired_mode;

    // mode specific switching actions
    switch (desired_mode) {
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
            m_speed_ramp_handler[j]->stop();
        break;
        case VOCAB_CM_MIXED:
            m_jntReferencePositions[j] = m_positions[j];
            m_trajectoryGenerationReferencePosition[j] = m_positions[j];
            m_jntReferenceVelocities[j] = 0.0;
             m_speed_ramp_handler[j]->stop();
            m_trajectory_generator[j]->setLimits(m_jointPosLimits[j].min,m_jointPosLimits[j].max);
            m_trajectory_generator[j]->initTrajectory(m_positions[j],m_trajectoryGenerationReferencePosition[j],m_trajectoryGenerationReferenceSpeed[j]);
        break;
        case VOCAB_CM_TORQUE :
        case VOCAB_CM_OPENLOOP :
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
    for (int i = 0; i < n_joint; i++)
        ret = ret && setControlMode(joints[i], modes[i]);
    return ret;
}

bool GazeboYarpControlBoardDriver::setControlModes(int *modes)
{
    bool ret = true;
    for (int i = 0; i < (int)m_numberOfJoints; i++)
        ret = ret && setControlMode(i, modes[i]);
    return ret;
}
