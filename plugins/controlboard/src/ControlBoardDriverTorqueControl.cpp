/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "ControlBoardDriver.h"


using namespace yarp::dev;

bool GazeboYarpControlBoardDriver::setRefTorque(int j, double t)
{
    if (j >= 0 && static_cast<size_t>(j) < m_numberOfJoints) {
        m_jntReferenceTorques[j] = t;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::setRefTorques(const double* t)
{
    if (!t) return false;
    for (size_t j = 0; j < m_numberOfJoints; ++j) {
        m_jntReferenceTorques[j] = t[j];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::setTorqueMode()
{
    bool ret = true;
    for (size_t j = 0; j < m_numberOfJoints; j++) {
        ret = ret && this->setControlMode(j, VOCAB_CM_TORQUE);
    }
    return ret;
}

bool GazeboYarpControlBoardDriver::getRefTorque(int j, double* t)
{
    if (t && j >= 0 && static_cast<size_t>(j) < m_numberOfJoints) {
        *t = m_jntReferenceTorques[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getRefTorques(double* t)
{
    if (!t) return false;
    for(size_t j = 0; j < m_numberOfJoints; ++j) {
        t[j] = m_jntReferenceTorques[j];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getTorque(int j, double* t)
{
    if (t && j >= 0 && static_cast<size_t>(j) < m_numberOfJoints) {
        *t = m_torques[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getTorques(double* t)
{
    if (!t) return false;
    for (size_t j = 0; j < m_numberOfJoints; ++j) {
        t[j] = m_torques[j];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getTorqueRange(int, double*, double *){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::getTorqueRanges(double *, double *){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::getBemfParam(int , double *){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::setBemfParam(int , double ){return false;} //NOT IMPLEMENTED
