/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "ControlBoardDriver.h"
#include <yarp/os/LogStream.h>
#include <yarp/os/LogComponent.h>


using namespace yarp::dev;


bool GazeboYarpControlBoardDriver::velocityMove(int j, double sp) //NOT TESTED
{
    if (j >= 0 && static_cast<size_t>(j) < m_numberOfJoints)
    {
        m_jntReferenceVelocities[j] = sp;
        m_velocity_watchdog[j]->reset();
        if (m_speed_ramp_handler[j])
        {
            m_speed_ramp_handler[j]->setReference(m_jntReferenceVelocities[j], m_trajectoryGenerationReferenceAcceleration[j]);
        }
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::velocityMove(const double *sp) //NOT TESTED
{
    if (!sp) return false;
    for (size_t i = 0; i < m_numberOfJoints; ++i)
    {
        velocityMove(i, sp[i]);
    }
    return true;
}

bool GazeboYarpControlBoardDriver::velocityMove(const int n_joint, const int *joints, const double *spds)
{
    if (!joints || !spds) return false;
    bool ret = true;
    for (int i = 0; i < n_joint && ret; i++)
    {
        ret = velocityMove(joints[i], spds[i]);
    }
    return ret;
}

bool GazeboYarpControlBoardDriver::getRefVelocity(const int joint, double *vel)
{
    if (vel && joint >= 0 && static_cast<size_t>(joint) < m_numberOfJoints)
    {
        *vel = m_jntReferenceVelocities[joint];
        return true;
    }
    return false;

}

bool GazeboYarpControlBoardDriver::getRefVelocities(double *vels)
{
    if (!vels) return false; //check or not check?
    bool ret = true;
    for (size_t i = 0; i < this->m_numberOfJoints && ret; i++) {
        ret = getRefVelocity(i, &vels[i]);
    }
    return ret;
}

bool GazeboYarpControlBoardDriver::getRefVelocities(const int n_joint, const int *joints, double *vels)
{
    if (!joints || !vels) return false; //check or not check?
    bool ret = true;
    for (int i = 0; i < n_joint && ret; i++) {
        ret = getRefVelocity(joints[i], &vels[i]);
    }
    return ret;
}
