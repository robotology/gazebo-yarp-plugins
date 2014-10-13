/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "gazebo_yarp_plugins/ControlBoardDriver.h"


using namespace yarp::dev;

bool GazeboYarpControlBoardDriver::positionMove(int j, double ref) //WORKS
{
    if (j >= 0 && j < (int)m_numberOfJoints) {
        m_trajectoryGenerationReferencePosition[j] = ref; //we will use this m_trajectoryGenerationReferencePosition in the next simulation onUpdate call to ask gazebo to set PIDs m_trajectoryGenerationReferencePosition to this value
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::stop(int j) //WORKS
{
    if (j >= 0 && j < (int)m_numberOfJoints) {
        m_trajectoryGenerationReferencePosition[j] = m_positions[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::stop() //WORKS
{
    m_trajectoryGenerationReferencePosition = m_positions;
    return true;
}

bool GazeboYarpControlBoardDriver::positionMove(const double *refs) //WORKS
{
    for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
        m_trajectoryGenerationReferencePosition[i] = refs[i];
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
    if (j >= 0 && j < (int)m_numberOfJoints) {
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
    for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
        m_trajectoryGenerationReferencePosition[i] = m_positions[i]+ deltas[i]; //TODO check if this is ok or m_trajectoryGenerationReferencePosition=m_trajectoryGenerationReferencePosition+delta!!!
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


bool GazeboYarpControlBoardDriver::setRefAcceleration(int j, double acc) //NOT IMPLEMENTED
{
    if (j >= 0 && j < (int)m_numberOfJoints) {
        m_trajectoryGenerationReferenceAcceleraton[j] = acc;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::setRefAccelerations(const double *accs) //NOT IMPLEMENTED
{
    for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
        m_trajectoryGenerationReferenceAcceleraton[i] = accs[i];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getRefAcceleration(int j, double *acc) //NOT IMPLEMENTED
{
    if (acc && j >= 0 && j < (int)m_numberOfJoints) {
        *acc = m_trajectoryGenerationReferenceAcceleraton[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getRefAccelerations(double *accs) //NOT IMPLEMENTED
{
    if (!accs) return false;
    for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
        accs[i] = m_trajectoryGenerationReferenceAcceleraton[i];
    }
    return true;
}

// IPositionControl2

bool GazeboYarpControlBoardDriver::positionMove(const int n_joint, const int *joints, const double *refs) //NOT IMPLEMENTED
{
    bool ret = true;
    for (int i = 0; i < n_joint; i++) {
        ret = ret && positionMove(joints[i], refs[i]);
    }
    return ret;
}

bool GazeboYarpControlBoardDriver::relativeMove(const int n_joint, const int *joints, const double *deltas) //NOT IMPLEMENTED
{
    return false;
}


bool GazeboYarpControlBoardDriver::checkMotionDone(const int n_joint, const int *joints, bool *flags) //NOT IMPLEMENTED
{
    return false;
}

bool GazeboYarpControlBoardDriver::setRefSpeeds(const int n_joint, const int *joints, const double *spds) //NOT IMPLEMENTED
{
    return false;
}


bool GazeboYarpControlBoardDriver::setRefAccelerations(const int n_joint, const int *joints, const double *accs) //NOT IMPLEMENTED
{
    return false;
}


bool GazeboYarpControlBoardDriver::getRefSpeeds(const int n_joint, const int *joints, double *spds) //NOT IMPLEMENTED
{
    return false;
}


bool GazeboYarpControlBoardDriver::getRefAccelerations(const int n_joint, const int *joints, double *accs) //NOT IMPLEMENTED
{
    return false;
}


bool GazeboYarpControlBoardDriver::stop(const int n_joint, const int *joints) //NOT IMPLEMENTED
{
    return false;
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
            m_referencePositions[j] = ref;
            return true;
        }
    } else {
        std::cerr << "[WARN] gazebo_yarp_controlboard: you tried to call setPosition" << std::endl;
        std::cerr << "[WARN] for a joint that is not in POSITION_DIRECT control mode." << std::endl;
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
