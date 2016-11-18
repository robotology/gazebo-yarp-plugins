/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "ControlBoardDriver.h"
#include <yarp/os/LogStream.h>


using namespace yarp::dev;

bool GazeboYarpControlBoardDriver::positionMove(int j, double ref) //WORKS
{
    if (j >= 0 && j < (int)m_numberOfJoints)
    {
        m_trajectoryGenerationReferencePosition[j] = ref; //we will use this m_trajectoryGenerationReferencePosition in the next simulation onUpdate call to ask gazebo to set PIDs m_trajectoryGenerationReferencePosition to this value
        m_trajectory_generator[j]->setLimits(m_jointPosLimits[j].min,m_jointPosLimits[j].max);
        m_trajectory_generator[j]->initTrajectory (m_positions[j], m_trajectoryGenerationReferencePosition[j], m_trajectoryGenerationReferenceSpeed[j]);
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::stop(int j) //WORKS
{
    if (j >= 0 && j < (int)m_numberOfJoints)
    {
        if (m_controlMode[j]==VOCAB_CM_POSITION)
        {
            m_trajectoryGenerationReferencePosition[j] = m_positions[j];
            m_trajectory_generator[j]->abortTrajectory(m_positions[j]);
        }
        else if  (m_controlMode[j]==VOCAB_CM_VELOCITY)
        {
            m_jntReferenceVelocities[j]=0;
            m_speed_ramp_handler[j]->stop();
        }
        else if  (m_controlMode[j]==VOCAB_CM_MIXED)
        {
            m_trajectoryGenerationReferencePosition[j] = m_positions[j];
            m_trajectory_generator[j]->abortTrajectory(m_positions[j]);
            m_jntReferenceVelocities[j]=0;
            m_speed_ramp_handler[j]->stop();
        }
        else if  (m_controlMode[j]==VOCAB_CM_POSITION_DIRECT)
        {
            m_trajectoryGenerationReferencePosition[j] = m_positions[j];
            m_jntReferencePositions[j] = m_positions[j];
        }
        else
        {
        }
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::stop() //WORKS
{
    for (unsigned int i = 0; i < m_numberOfJoints; ++i)
    {
        stop(i);
    }
    return true;
}

bool GazeboYarpControlBoardDriver::positionMove(const double *refs) //WORKS
{
    for (unsigned int i = 0; i < m_numberOfJoints; ++i)
    {
        positionMove(i,refs[i]);
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getAxes(int *ax) // WORKS
{
    if (!ax) return false;
    *ax = m_numberOfJoints;
    return true;
}

bool GazeboYarpControlBoardDriver::setRefSpeed(int j, double sp) //WORKS
{
    if (j >= 0 && j < (int)m_numberOfJoints)
    {
        m_trajectoryGenerationReferenceSpeed[j] = sp;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getRefSpeed(int j, double *ref) //WORKS
{
    if (ref && j >= 0 && j < (int)m_numberOfJoints) {
        *ref = m_trajectoryGenerationReferenceSpeed[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getRefSpeeds(double *spds) //WORKS
{
    if (!spds) return false;
    for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
        spds[i] = m_trajectoryGenerationReferenceSpeed[i];
    }
    return true;
}



bool GazeboYarpControlBoardDriver::relativeMove(int j, double delta) //NOT TESTED
{
    if (j >= 0 && j < (int)m_numberOfJoints) {
        m_trajectoryGenerationReferencePosition[j] = m_positions[j] + delta; //TODO check if this is ok or m_trajectoryGenerationReferencePosition=m_trajectoryGenerationReferencePosition+delta!!!
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::relativeMove(const double *deltas) //NOT TESTED
{
    for (unsigned int i = 0; i < m_numberOfJoints; ++i)
    {
        relativeMove(i,deltas[i]);
    }
    return true;
}

bool GazeboYarpControlBoardDriver::checkMotionDone(int j, bool *flag) //NOT TESTED
{
    if (flag && j >= 0 && j < (int)m_numberOfJoints) {
        *flag = m_isMotionDone[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::checkMotionDone(bool *flag) //NOT TESTED
{
    if (!flag) return false;
    bool temp_flag = true;
    //*flag=true;
    for (unsigned int j = 0; j < m_numberOfJoints; ++j) {
        temp_flag = temp_flag && m_isMotionDone[j];
    }
    *flag = temp_flag;
    return true;
}

bool GazeboYarpControlBoardDriver::setPositionMode() //NOT TESTED
{
    bool ret = true;
    for (unsigned int j = 0; j < m_numberOfJoints; j++) {
        ret = ret && this->setControlMode(j, VOCAB_CM_POSITION);
    }
    return ret;
}

bool GazeboYarpControlBoardDriver::setRefSpeeds(const double *spds) //NOT TESTED
{
    for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
        m_trajectoryGenerationReferenceSpeed[i] = spds[i];
    }
    return true;
}


bool GazeboYarpControlBoardDriver::setRefAcceleration(int j, double acc) 
{
    if (j >= 0 && j < (int)m_numberOfJoints) {
        m_trajectoryGenerationReferenceAcceleration[j] = acc;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::setRefAccelerations(const double *accs) 
{
    for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
        m_trajectoryGenerationReferenceAcceleration[i] = accs[i];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getRefAcceleration(int j, double *acc) 
{
    if (acc && j >= 0 && j < (int)m_numberOfJoints) {
        *acc = m_trajectoryGenerationReferenceAcceleration[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getRefAccelerations(double *accs) 
{
    if (!accs) return false;
    for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
        accs[i] = m_trajectoryGenerationReferenceAcceleration[i];
    }
    return true;
}

// IPositionControl2

bool GazeboYarpControlBoardDriver::positionMove(const int n_joint, const int *joints, const double *refs)
{
    if (!joints || !refs) return false;
    bool ret = true;
    for (int i = 0; i < n_joint && ret; i++)
    {
        ret = positionMove(joints[i], refs[i]);
    }
    return ret;
}

bool GazeboYarpControlBoardDriver::relativeMove(const int n_joint, const int *joints, const double *deltas)
{
    if (!joints || !deltas) return false; //check or not check?
    bool ret = true;
    for (int i = 0; i < n_joint && ret; i++)
    {
        ret = relativeMove(joints[i], deltas[i]);
    }
    return ret;
}

bool GazeboYarpControlBoardDriver::checkMotionDone(const int n_joint, const int *joints, bool *flags)
{
    if (!joints || !flags) return false;
    bool ret = true;
    for (int i = 0; i < n_joint && ret; i++) {
        ret = checkMotionDone(joints[i], &flags[i]);
    }
    return ret;
}

bool GazeboYarpControlBoardDriver::setRefSpeeds(const int n_joint, const int *joints, const double *spds) //NOT IMPLEMENTED
{
    if (!joints || !spds) return false; //check or not check?
    bool ret = true;
    for (int i = 0; i < n_joint && ret; i++) {
        ret = setRefSpeed(joints[i], spds[i]);
    }
    return ret;
}


bool GazeboYarpControlBoardDriver::setRefAccelerations(const int n_joint, const int *joints, const double *accs)
{
    if (!joints || !accs) return false; //check or not check?
    bool ret = true;
    for (int i = 0; i < n_joint && ret; i++) {
        ret = setRefAcceleration(joints[i], accs[i]);
    }
    return ret;
}


bool GazeboYarpControlBoardDriver::getRefSpeeds(const int n_joint, const int *joints, double *spds)
{
    if (!joints || !spds) return false; //check or not check?
    bool ret = true;
    for (int i = 0; i < n_joint && ret; i++) {
        ret = getRefSpeed(joints[i], &spds[i]);
    }
    return ret;
}


bool GazeboYarpControlBoardDriver::getRefAccelerations(const int n_joint, const int *joints, double *accs)
{
    if (!joints || !accs) return false; //check or not check?
    bool ret = true;
    for (int i = 0; i < n_joint && ret; i++) {
        ret = getRefAcceleration(joints[i], &accs[i]);
    }
    return ret;
}


bool GazeboYarpControlBoardDriver::stop(const int n_joint, const int *joints) //NOT IMPLEMENTED
{
    if (!joints) return false; //check or not check?
    bool ret = true;
    for (int i = 0; i < n_joint; i++) {
        if (joints[i] >= 0 && joints[i] < (int)m_numberOfJoints) {
            ret = ret && stop(joints[i]);
        } else {
            ret = false;
            break;
        }
    }
    return ret;
}


// IPOSITION DIRECT
bool GazeboYarpControlBoardDriver::setPositionDirectMode()
{
    bool ret = true;
    for (int j = 0; j < (int)m_numberOfJoints; j++)  {
        ret = ret && this->setControlMode(j, VOCAB_CM_POSITION_DIRECT);
    }
    return ret;
}

bool GazeboYarpControlBoardDriver::setPosition(int j, double ref)
{
    if (m_controlMode[j] == VOCAB_CM_POSITION_DIRECT) {
        if (j >= 0 && j < (int)m_numberOfJoints) {
            m_jntReferencePositions[j] = ref;
            return true;
        }
    } else {
        yError() << "gazebo_yarp_controlboard: you tried to call setPosition " <<
        "for a joint that is not in POSITION_DIRECT control mode.";
    }
    return false;
}

bool GazeboYarpControlBoardDriver::setPositions(const int n_joint, const int *joints, double *refs)
{
    bool ret = true;
    for (int i = 0; i < n_joint; i++) {
        ret = ret && setPosition(joints[i], refs[i]);
    }
    return ret;
}

bool GazeboYarpControlBoardDriver::setPositions(const double *refs)
{
    bool ret = true;
    for (int j = 0; j < this->m_numberOfJoints; j++ ) {
        ret = ret && setPosition(j, refs[j]);
    }
    return ret;
}

bool GazeboYarpControlBoardDriver::getTargetPosition(const int joint, double *ref)
{
    if (ref && joint >= 0 && joint < (int)m_numberOfJoints)
    {
      *ref = m_trajectoryGenerationReferencePosition[joint];
      return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getTargetPositions(double *refs)
{
    if (!refs) return false; //check or not check?
    bool ret = true;
    for (int i = 0; i < this->m_numberOfJoints && ret; i++) {
        ret = getTargetPosition(i, &refs[i]);
    }
    return ret;
}

bool GazeboYarpControlBoardDriver::getTargetPositions(const int n_joint, const int *joints, double *refs)
{
    if (!joints || !refs) return false; //check or not check?
    bool ret = true;
    for (int i = 0; i < n_joint && ret; i++) {
        ret = getTargetPosition(joints[i], &refs[i]);
    }
    return ret;
}
    