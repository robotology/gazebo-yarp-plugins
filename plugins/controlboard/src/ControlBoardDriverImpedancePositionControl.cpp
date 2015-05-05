/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "ControlBoardDriver.h"

using namespace yarp::dev;

/*
 * Get Stiffness in [Nm/deg] and damping in [Nm*sec/deg]
 */
bool GazeboYarpControlBoardDriver::getImpedance(int j, double *stiffness, double *damping)
{
    if (j >= 0 && j < m_numberOfJoints) {
        *stiffness = m_impedancePosPDs[j].p;
        *damping = m_impedancePosPDs[j].d;
        return true;
    }
    return false;
}

/*
 * Set Stiffness in [Nm/deg] and damping in [Nm*sec/deg]
 */
bool GazeboYarpControlBoardDriver::setImpedance(int j, double stiffness, double damping)
{
    if (j >= 0 && j < m_numberOfJoints) {
        m_impedancePosPDs[j].p = stiffness;
        m_impedancePosPDs[j].d = damping;
        return true;
    }
    return false;
}

/*
 * Set torque offset in [Nm]
 */
bool GazeboYarpControlBoardDriver::setImpedanceOffset(int j, double offset)
{
    if(j >= 0 && j < m_numberOfJoints)
    {
        m_torqueOffsett[j] = offset;
        return true;
    }
    return false;
}

/*
 * Get torque offset in [Nm]
 */
bool GazeboYarpControlBoardDriver::getImpedanceOffset(int j, double* offset)
{
    if(j >= 0 && j < m_numberOfJoints)
    {
        *offset = m_torqueOffsett[j];
        return true;
    }
    return false;
}

/*
 * Set minimum and maximum stiffness in [Nm/deg] and damping in [Nm*sec/deg]
 */
bool GazeboYarpControlBoardDriver::getCurrentImpedanceLimit(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp)
{
    if(j >= 0 && j < m_numberOfJoints)
    {
        //Hardcoded numbers...just to try
        *min_stiff = m_minStiffness[j];
        *max_stiff = m_maxStiffness[j];
        *min_damp = m_minDamping[j];
        *max_damp = m_maxDamping[j];
        return true;
    }
    return false;
}
