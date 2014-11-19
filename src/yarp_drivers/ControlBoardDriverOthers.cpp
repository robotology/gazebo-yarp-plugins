/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "ControlBoardDriver.h"

using namespace yarp::dev;

// IControlLimits
bool GazeboYarpControlBoardDriver::getLimits(int axis, double *min, double *max) //WORKS
{
    if (!min || !max) return false;
    *min = m_jointLimits[axis].min;
    *max = m_jointLimits[axis].max;
    return true;
}

bool GazeboYarpControlBoardDriver::setLimits(int axis, double min, double max) //WORKS
{
    if (axis < 0 || axis >= (int)m_numberOfJoints) return false;
    m_jointLimits[axis].max = max;
    m_jointLimits[axis].min = min;
    return true;
}

// IControlLimits2
bool GazeboYarpControlBoardDriver::getVelLimits(int axis, double* min, double* max) //NOT TESTED
{
    return false;
}

bool GazeboYarpControlBoardDriver::setVelLimits(int axis, double min, double max) //NOT TESTED
{
    return false;
}

//Amplifiers
bool GazeboYarpControlBoardDriver::enableAmp(int j) //NOT IMPLEMENTED
{
    if (j >= 0 && j < (int)m_numberOfJoints) {
        amp[j] = 1;
        m_controlMode[j] = VOCAB_CM_POSITION;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::disableAmp(int j) //NOT IMPLEMENTED
{
    if (j >= 0 && j < (int)m_numberOfJoints) {
        amp[j] = 0;
        m_controlMode[j] = VOCAB_CM_IDLE;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getCurrent(int j, double* val) //NOT IMPLEMENTED
{
    if (val && j >= 0 && j < (int)m_numberOfJoints) {
        *val = amp[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getCurrents(double *vals) //NOT IMPLEMENTED
{
    if (!vals) return false;
    for (unsigned int i=0; i<m_numberOfJoints; i++) {
        vals[i] = amp[i];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::setMaxCurrent(int, double) //NOT IMPLEMENTED
{
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

bool GazeboYarpControlBoardDriver::calibrate2(int j, unsigned int iv, double v1, double v2, double v3) //NOT IMPLEMENTED
{
    fprintf(stderr, "fakebot: calibrating joint %d with parameters %u %f %f %f\n", j, iv, v1, v2, v3);
    return true;
}

bool GazeboYarpControlBoardDriver::done(int j) // NOT IMPLEMENTED
{
    fprintf(stderr , "fakebot: calibration done on joint %d.\n", j);
    return true;
}
