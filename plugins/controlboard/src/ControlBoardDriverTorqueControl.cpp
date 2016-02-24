/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "ControlBoardDriver.h"


using namespace yarp::dev;

bool GazeboYarpControlBoardDriver::setRefTorque(int j, double t)
{
    if (j >= 0 && j < (int)m_numberOfJoints) {
        m_jntReferenceTorques[j] = t;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::setRefTorques(const double* t)
{
    if (!t) return false;
    for (unsigned int j = 0; j < m_numberOfJoints; ++j) {
        m_jntReferenceTorques[j] = t[j];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::setTorqueMode()
{
    bool ret = true;
    for (unsigned int j = 0; j < m_numberOfJoints; j++) {
        ret = ret && this->setControlMode(j, VOCAB_CM_TORQUE);
    }
    return ret;
}

bool GazeboYarpControlBoardDriver::getRefTorque(int j, double* t)
{
    if (t && j >= 0 && j < (int)m_numberOfJoints) {
        *t = m_jntReferenceTorques[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getRefTorques(double* t)
{
    if (!t) return false;
    for(unsigned int j = 0; j < m_numberOfJoints; ++j) {
        t[j] = m_jntReferenceTorques[j];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getTorque(int j, double* t)
{
    if (t && j >= 0 && j < (int)m_numberOfJoints) {
        *t = m_torques[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getTorques(double* t)
{
    if (!t) return false;
    for (unsigned int j = 0; j < m_numberOfJoints; ++j) {
        t[j] = m_torques[j];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::setTorquePid(int joint, const Pid &pid)
{
    if (joint < 0 || joint >= m_numberOfJoints) return false;
    PID &currentPid = m_torquePIDs[joint];
    currentPid.p = pid.kp;
    currentPid.d = pid.kd;
    currentPid.i = pid.ki;
    return true;
}

bool GazeboYarpControlBoardDriver::setTorquePids(const Pid *newPids)
{
    for (unsigned i = 0; i < m_numberOfJoints; ++i) {
        PID &currentPid = m_torquePIDs[i];
        currentPid.p = newPids[i].kp;
        currentPid.d = newPids[i].kd;
        currentPid.i = newPids[i].ki;
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getTorquePid(int joint, Pid *pid)
{
    if (joint < 0 || joint >= m_numberOfJoints) return false;
    if (!pid) return false;
    PID &currentPid = m_torquePIDs[joint];
    pid->kp = currentPid.p;
    pid->kd = currentPid.d;
    pid->ki = currentPid.i;
    return true;
}

bool GazeboYarpControlBoardDriver::getTorquePids(Pid *pids)
{
    if (!pids) return false;
    for (unsigned j = 0; j < m_numberOfJoints; ++j) {
        PID &currentPid = m_torquePIDs[j];
        pids[j].kp = currentPid.p;
        pids[j].kd = currentPid.d;
        pids[j].ki = currentPid.i;
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getTorqueRange(int, double*, double *){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::getTorqueRanges(double *, double *){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::setTorqueErrorLimit(int , double ){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::setTorqueErrorLimits(const double *){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::getTorqueError(int , double *){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::getTorqueErrors(double *){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::getTorquePidOutput(int , double *){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::getTorquePidOutputs(double *){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::getTorqueErrorLimit(int , double *){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::getTorqueErrorLimits(double *){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::resetTorquePid(int ){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::disableTorquePid(int ){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::enableTorquePid(int ){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::setTorqueOffset(int , double ){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::getBemfParam(int , double *){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::setBemfParam(int , double ){return false;} //NOT IMPLEMENTED
