/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include <iostream>
#include "FakeControlBoardDriver.h"

namespace yarp {
namespace dev {

bool GazeboYarpFakeControlBoardDriver::getOutput(int j, double *v)
{
    if (v && j >= 0 && j < (int)m_numberOfJoints) {
        *v = m_torques[j];
        return true;
    }
    return false;
}

bool GazeboYarpFakeControlBoardDriver::getOutputs(double *v)
{
    if (!v) return false;
    for (unsigned int j = 0; j < m_numberOfJoints; ++j) {
        v[j] = m_torques[j];
    }
    return true;
}

bool GazeboYarpFakeControlBoardDriver::setRefOutput(int j, double v) {return false;}
bool GazeboYarpFakeControlBoardDriver::setRefOutputs(const double* v) {return false;} 
bool GazeboYarpFakeControlBoardDriver::getRefOutput(int j, double *v) {return false;}
bool GazeboYarpFakeControlBoardDriver::getRefOutputs(double *v) {return false;}
bool GazeboYarpFakeControlBoardDriver::setOpenLoopMode() {return false;}
}
}
