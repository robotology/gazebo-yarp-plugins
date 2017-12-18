/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "ControlBoardDriver.h"
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>


using namespace yarp::dev;

bool GazeboYarpControlBoardDriver::setRefTorque(int j, double t)
{
    if (!checkIfTorqueIsValid(t))
        return false;

    if (j >= 0 && static_cast<size_t>(j) < m_numberOfJoints) {
        m_jntReferenceTorques[j] = t;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::setRefTorques(const double* t)
{
    if (!t) return false;

    if (!checkIfTorqueIsValid(t))
        return false;

    for (size_t j = 0; j < m_numberOfJoints; ++j) {
        m_jntReferenceTorques[j] = t[j];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::setRefTorques(const int n_joint, const int *joints, const double *t)
{
    if (!joints || !t) return false;

    if (!checkIfTorqueIsValid(t))
        return false;

    bool ret = true;
    for (int i = 0; i < n_joint && ret; i++) {
        m_jntReferenceTorques[joints[i]] = t[i];
    }
    return ret;
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

bool GazeboYarpControlBoardDriver::getTorqueRange(int, double*, double *){return false;}
bool GazeboYarpControlBoardDriver::getTorqueRanges(double *, double *){return false;}
bool GazeboYarpControlBoardDriver::getBemfParam(int , double *){return false;}
bool GazeboYarpControlBoardDriver::setBemfParam(int , double ){return false;}
bool GazeboYarpControlBoardDriver::getMotorTorqueParams(int ,  yarp::dev::MotorTorqueParameters *){return false;}
bool GazeboYarpControlBoardDriver::setMotorTorqueParams(int , const yarp::dev::MotorTorqueParameters ){return false;}

bool GazeboYarpControlBoardDriver::checkIfTorqueIsValid(const double* torques) const
{
    if (!torques)
        return false;

    bool out=true;
    for (int index=0;index<m_numberOfJoints;++index)
    {
        out=out && checkIfTorqueIsValid(torques[index]);
    }        
    return out;
}

bool GazeboYarpControlBoardDriver::checkIfTorqueIsValid(double torque) const
{
    if (std::isnan(torque) || std::isinf(torque))
    {
        yError() << "GazeboYarpControlBoard : controlBoard  invalid torque value:" << torque;
        return false;
    }
    return true;
}
