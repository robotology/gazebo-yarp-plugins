/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "ControlBoardDriver.h"

using namespace yarp::dev;

bool GazeboYarpControlBoardDriver::getAxisName(int axis, std::string& name)
{
    if (axis < 0 || static_cast<size_t>(axis) >= m_numberOfJoints) return false;

    for (unsigned int cpl_cnt = 0; cpl_cnt < m_coupling_handler.size(); cpl_cnt++)
    {
        if (m_coupling_handler[cpl_cnt])
        {
            if (m_coupling_handler[cpl_cnt]->checkJointIsCoupled(axis))
            {
                name = m_coupling_handler[cpl_cnt]->getCoupledJointName(axis);
                return true;
            }
        }
    }
    name = std::string(controlboard_joint_names.at(axis));
    return true;
}

bool GazeboYarpControlBoardDriver::getJointType(int axis, yarp::dev::JointTypeEnum& type)
{
    if (axis < 0 || static_cast<size_t>(axis) >= m_numberOfJoints) return false;
    if (this->m_jointTypes[axis] == JointType_Revolute) {
        type = yarp::dev::VOCAB_JOINTTYPE_REVOLUTE;
    } else if (this->m_jointTypes[axis] == JointType_Prismatic) {
        type = yarp::dev::VOCAB_JOINTTYPE_PRISMATIC;
    } else {
        type = yarp::dev::VOCAB_JOINTTYPE_UNKNOWN;
    }

    std::string(controlboard_joint_names.at(axis));
    return true;
}

// IControlLimits
bool GazeboYarpControlBoardDriver::getLimits(int axis, double *min, double *max) //WORKS
{
    if (axis < 0 || static_cast<size_t>(axis) >= m_numberOfJoints) return false;
    if (!min || !max) return false;
    *min = m_jointPosLimits[axis].min;
    *max = m_jointPosLimits[axis].max;
    return true;
}

bool GazeboYarpControlBoardDriver::setLimits(int axis, double min, double max) //WORKS
{
    if (axis < 0 || static_cast<size_t>(axis) >= m_numberOfJoints) return false;
    m_jointPosLimits[axis].max = max;
    m_jointPosLimits[axis].min = min;
    return true;
}

// IControlLimits
bool GazeboYarpControlBoardDriver::getVelLimits(int axis, double* min, double* max) //WORKS
{
    if (axis < 0 || static_cast<size_t>(axis) >= m_numberOfJoints) return false;
    if (!min || !max) return false;
    *min = m_jointVelLimits[axis].min;
    *max = m_jointVelLimits[axis].max;
    return true;
}

bool GazeboYarpControlBoardDriver::setVelLimits(int axis, double min, double max) //WORKS
{
    if (axis < 0 || static_cast<size_t>(axis) >= m_numberOfJoints) return false;
    m_jointVelLimits[axis].max = max;
    m_jointVelLimits[axis].min = min;
    return true;
}


//Amplifiers
bool GazeboYarpControlBoardDriver::enableAmp(int j) //NOT IMPLEMENTED
{
    if (j >= 0 && static_cast<size_t>(j) < m_numberOfJoints) {
        m_amp[j] = 1;
        m_controlMode[j] = VOCAB_CM_POSITION;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::disableAmp(int j) //NOT IMPLEMENTED
{
    if (j >= 0 && static_cast<size_t>(j) < m_numberOfJoints) {
        m_amp[j] = 0;
        m_controlMode[j] = VOCAB_CM_IDLE;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::setMaxCurrent(int, double) //NOT IMPLEMENTED
{
    return true;
}

bool GazeboYarpControlBoardDriver::getMaxCurrent(int j, double *v) //NOT IMPLEMENTED
{
    if (!v) return false;
    *v = 0;
    return true;
}

bool GazeboYarpControlBoardDriver::getAmpStatus(int *st) //NOT IMPLEMENTED
{
    if (!st) return false;
    *st = 0;
    return true;
}

bool GazeboYarpControlBoardDriver::getAmpStatus(int, int *v) //NOT IMPLEMENTED
{
    if (!v) return false;
    *v = 0;
    return true;
}

bool GazeboYarpControlBoardDriver::calibrateAxisWithParams(int j, unsigned int iv, double v1, double v2, double v3) //NOT IMPLEMENTED
{
    yDebug("fakebot: calibrating joint %d with parameters %u %f %f %f\n", j, iv, v1, v2, v3);
    return true;
}

bool GazeboYarpControlBoardDriver::calibrationDone(int j) // NOT IMPLEMENTED
{
    yDebug("fakebot: calibration done on joint %d.\n", j);
    return true;
}
