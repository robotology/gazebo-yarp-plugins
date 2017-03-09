/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "FakeControlBoardDriver.h"

using namespace yarp::dev;

bool GazeboYarpFakeControlBoardDriver::getAxisName(int axis, yarp::os::ConstString& name)
{
    if (axis < 0 || axis >= (int)m_numberOfJoints) return false;
    name = yarp::os::ConstString(m_jointNames.at(axis));
    return true;
}

bool GazeboYarpFakeControlBoardDriver::getJointType(int axis, yarp::dev::JointTypeEnum& type)
{
    if (axis < 0 || axis >= (int)m_numberOfJoints) return false;
    
    type = this->m_jointTypes[axis];
  
    return true;
}

// IControlLimits
bool GazeboYarpFakeControlBoardDriver::getLimits(int axis, double *min, double *max) { getEncoder(axis,max); return getEncoder(axis,min);}
bool GazeboYarpFakeControlBoardDriver::getVelLimits(int axis, double* min, double* max) { getZero(axis,max); return getZero(axis,min);}

bool GazeboYarpFakeControlBoardDriver::setLimits(int axis, double min, double max) {return false;}
bool GazeboYarpFakeControlBoardDriver::setVelLimits(int axis, double min, double max) {return false;}
bool GazeboYarpFakeControlBoardDriver::enableAmp(int j) {return false;}
bool GazeboYarpFakeControlBoardDriver::disableAmp(int j) {return false;}
bool GazeboYarpFakeControlBoardDriver::getCurrent(int j, double* val) {return false;}
bool GazeboYarpFakeControlBoardDriver::getCurrents(double *vals) {return false;}
bool GazeboYarpFakeControlBoardDriver::setMaxCurrent(int, double) {return false;}
bool GazeboYarpFakeControlBoardDriver::getMaxCurrent(int j, double *v) {return false;}
bool GazeboYarpFakeControlBoardDriver::getAmpStatus(int *st) {return false;}
bool GazeboYarpFakeControlBoardDriver::getAmpStatus(int, int *v) {return false;}
bool GazeboYarpFakeControlBoardDriver::calibrate2(int j, unsigned int iv, double v1, double v2, double v3) {return false;}
bool GazeboYarpFakeControlBoardDriver::done(int j) {return false;}


