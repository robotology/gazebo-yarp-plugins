/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "FakeControlBoardDriver.h"

using namespace yarp::dev;

bool GazeboYarpFakeControlBoardDriver::getAxisName(int axis, std::string& name)
{
    if (axis < 0 || axis >= (int)m_numberOfJoints) return false;
    name = std::string(m_jointNames.at(axis));
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
bool GazeboYarpFakeControlBoardDriver::calibrateAxisWithParams(int j, unsigned int iv, double v1, double v2, double v3) {return false;}
bool GazeboYarpFakeControlBoardDriver::calibrationDone(int j) {return false;}

// PWM interface
bool GazeboYarpFakeControlBoardDriver::getNumberOfMotors(int *ax) { return getAxes(ax); }
bool GazeboYarpFakeControlBoardDriver::setRefDutyCycle(int j, double v) {return false;}
bool GazeboYarpFakeControlBoardDriver::setRefDutyCycles(const double *v) {return false;}
bool GazeboYarpFakeControlBoardDriver::getRefDutyCycle(int j, double *v) {return false;}
bool GazeboYarpFakeControlBoardDriver::getRefDutyCycles(double *v) {return false;}
bool GazeboYarpFakeControlBoardDriver::getDutyCycle(int j, double *v) {return false;}
bool GazeboYarpFakeControlBoardDriver::getDutyCycles(double *v) {return false;}

// Current interface
bool GazeboYarpFakeControlBoardDriver::getCurrentRange(int j, double *min, double *max) {return false;}
bool GazeboYarpFakeControlBoardDriver::getCurrentRanges(double *min, double *max) {return false;}
bool GazeboYarpFakeControlBoardDriver::setRefCurrents(const double *t) {return false;}
bool GazeboYarpFakeControlBoardDriver::setRefCurrent(int j, double t) {return false;}
bool GazeboYarpFakeControlBoardDriver::setRefCurrents(const int n_joint, const int *joints, const double *t) { return (n_joint == 0); }
bool GazeboYarpFakeControlBoardDriver::getRefCurrents(double *t) {return false;}
bool GazeboYarpFakeControlBoardDriver::getRefCurrent(int j, double *t) {return false;}
