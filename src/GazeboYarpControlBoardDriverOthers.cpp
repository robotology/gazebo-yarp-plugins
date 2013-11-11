/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <GazeboYarpControlBoardDriver.h>

using namespace yarp::dev;

// IControlLimits
bool GazeboYarpControlBoardDriver::getLimits(int axis, double *min, double *max) //NOT TESTED
{
    *min=min_pos[axis];
    *max=max_pos[axis];
    return true;
}

bool GazeboYarpControlBoardDriver::setLimits(int axis, double min, double max) //NOT TESTED
{
    max_pos[axis]=max;
    min_pos[axis]=min;
    return true;
}

//Amplifiers
bool GazeboYarpControlBoardDriver::enableAmp(int j) //NOT IMPLEMENTED
{
    if (j<_robot_number_of_joints) {
        amp[j] = 1;
        control_mode[j]=VOCAB_CM_POSITION;
    }
    return true;
}

bool GazeboYarpControlBoardDriver::disableAmp(int j) //NOT IMPLEMENTED
{
    if (j<_robot_number_of_joints) {
        amp[j] = 0;
        control_mode[j]=VOCAB_CM_IDLE;
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getCurrent(int j, double *val) //NOT IMPLEMENTED
{
    if (j<_robot_number_of_joints) {
        val[j] = amp[j];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getCurrents(double *vals) //NOT IMPLEMENTED
{
    for (unsigned int i=0; i<_robot_number_of_joints; i++) {
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
    *st = 0;
    return true;
}

bool GazeboYarpControlBoardDriver::getAmpStatus(int, int *v) //NOT IMPLEMENTED
{
    *v=0;
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

