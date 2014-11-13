/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include "ControlBoardDriver.h"

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/transport/Publisher.hh>

#include <yarp/os/Vocab.h>


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
          || mode == VOCAB_CM_OPENLOOP
          || mode == VOCAB_CM_IDLE
          || mode == VOCAB_CM_FORCE_IDLE)) {
        std::cerr << "[WARN] request control mode "
                  << yarp::os::Vocab::decode(mode) << " that is not supported by "
                  << " gazebo_yarp_controlboard plugin." << std::endl;
        return false;
    }
    
    
    int desired_mode = mode;
    
    // Convert VOCAB_CM_FORCE_IDLE (that as a special meaning in real robots 
    //   subjects to hardware fault) to VOCAB_CM_IDLE
    // This is necessary for having a working "idle" button in the robotMotorGui
    if (mode == VOCAB_CM_FORCE_IDLE) {
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
        case VOCAB_CM_POSITION_DIRECT :
            m_referencePositions[j] = m_positions[j];
            m_trajectoryGenerationReferencePosition[j] = m_positions[j];
        break;
        case VOCAB_CM_VELOCITY :
            m_referenceVelocities[j] = 0.0;
        break;
        case VOCAB_CM_TORQUE :
        case VOCAB_CM_OPENLOOP :
            m_referenceTorques[j] = m_torques[j];
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
