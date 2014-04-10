/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "gazebo_yarp_plugins/ControlBoardDriver.h"

using namespace yarp::dev;

bool GazeboYarpControlBoardDriver::getImpedance(int j, double *stiffness, double *damping)
{
    if(j >= 0 && j < numberOfJoints)
    {
        *stiffness = _impedancePosPDs[j].p;
        *damping = _impedancePosPDs[j].d;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::setImpedance(int j, double stiffness, double damping)
{
    if(j >= 0 && j < numberOfJoints)
    {
        _impedancePosPDs[j].p = stiffness;
        _impedancePosPDs[j].d = damping;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::setImpedanceOffset(int j, double offset)
{
    if(j >= 0 && j < numberOfJoints)
    {
        torqueOffsett[j] = offset;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getImpedanceOffset(int j, double* offset)
{
    if(j >= 0 && j < numberOfJoints)
    {
        *offset = torqueOffsett[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getCurrentImpedanceLimit(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp)
{
    if(j >= 0 && j < numberOfJoints)
    {
        //Hardcoded numbers...just to try
        *min_stiff = minStiffness[j];
        *max_stiff = maxStiffness[j];
        *min_damp = minDamping[j];
        *max_damp = maxDamping[j];
        return true;
    }
    return false;
}
