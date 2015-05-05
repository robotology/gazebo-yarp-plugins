/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include "ControlBoardDriver.h"


using namespace yarp::dev;


bool GazeboYarpControlBoardDriver::setVelocityMode() //NOT TESTED
{
    bool ret = true;
    for (unsigned int j = 0; j < m_numberOfJoints; j++) {
        ret = ret && this->setControlMode(j, VOCAB_CM_VELOCITY);
    }
    return ret;
}

bool GazeboYarpControlBoardDriver::velocityMove(int j, double sp) //NOT TESTED
{
    if (j >= 0 && j < (int)m_numberOfJoints) {
        m_referenceVelocities[j] = sp;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::velocityMove(const double *sp) //NOT TESTED
{
    if (!sp) return false;
    for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
        m_referenceVelocities[i] = sp[i];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::velocityMove(const int n_joint, const int *joints, const double *spds)
{
    if (!joints || !spds) return false;
    bool ret = true;
    for (int i = 0; i < n_joint && ret; i++) {
        ret = velocityMove(joints[i], spds[i]);
    }
    return ret;
}

bool GazeboYarpControlBoardDriver::setVelPid(int j, const yarp::dev::Pid &pid)
{
    if (j >= 0 && j < (int)m_numberOfJoints) {
        PID newPid = { pid.kp, pid.ki, pid.kd, pid.max_int, pid.max_output };
        m_velocityPIDs[j] = newPid;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::setVelPids(const yarp::dev::Pid *pids)
{
    if (!pids) return false;
    for (unsigned j = 0; j < m_numberOfJoints; j++) {
        PID newPid = { pids[j].kp, pids[j].ki, pids[j].kd, pids[j].max_int, pids[j].max_output };
        m_velocityPIDs[j] = newPid;
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getVelPid(int j, yarp::dev::Pid *pid)
{
    if (!pid) return false;
    if (j >= 0 && j < (int)m_numberOfJoints) {
        PID currentPID = m_velocityPIDs[j];
        pid->kp = currentPID.p;
        pid->ki = currentPID.i;
        pid->kd = currentPID.d;
        pid->max_int = currentPID.maxInt;
        pid->max_output = currentPID.maxOut;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getVelPids(yarp::dev::Pid *pids)
{
    if (!pids) return false;
    for (unsigned j = 0; j < m_numberOfJoints; j++) {
        PID currentPID = m_velocityPIDs[j];
        pids[j].kp = currentPID.p;
        pids[j].ki = currentPID.i;
        pids[j].kd = currentPID.d;
        pids[j].max_int = currentPID.maxInt;
        pids[j].max_output = currentPID.maxOut;
    }
    return true;
}
