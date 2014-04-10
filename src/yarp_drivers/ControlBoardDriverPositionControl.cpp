/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "gazebo_yarp_plugins/ControlBoardDriver.h"


using namespace yarp::dev;

bool GazeboYarpControlBoardDriver::positionMove(int j, double ref) //WORKS
{
//    std::cout << " positionMove" << j << ref;
    if (j >= 0 && j < (int)numberOfJoints) {
        referencePosition[j] = ref; //we will use this referencePosition in the next simulation onUpdate call to ask gazebo to set PIDs referencePosition to this value
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::stop(int j) //WORKS
{
    if (j >= 0 && j < (int)numberOfJoints) {
        referencePosition[j] = pos[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::stop() //WORKS
{
    referencePosition=pos;
    return true;
}

bool GazeboYarpControlBoardDriver::positionMove(const double *refs) //WORKS
{
    for (unsigned int i = 0; i < numberOfJoints; ++i) {
        referencePosition[i] = refs[i];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getAxes(int *ax) // WORKS
{
    if (!ax) return false;
    *ax = numberOfJoints;
    return true;
}

bool GazeboYarpControlBoardDriver::setRefSpeed(int j, double sp) //WORKS
{
    if (j >= 0 && j < (int)numberOfJoints) {
        referenceSpeed[j] = sp;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getRefSpeed(int j, double *ref) //WORKS
{
    if (ref && j >= 0 && j < (int)numberOfJoints) {
        *ref = referenceSpeed[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getRefSpeeds(double *spds) //WORKS
{
    if (!spds) return false;
    for (unsigned int i = 0; i < numberOfJoints; ++i) {
        spds[i] = referenceSpeed[i];
    }
    return true;
}



bool GazeboYarpControlBoardDriver::relativeMove(int j, double delta) //NOT TESTED
{
    if (j >= 0 && j < (int)numberOfJoints) {
        referencePosition[j] = pos[j] + delta; //TODO check if this is ok or referencePosition=referencePosition+delta!!!
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::relativeMove(const double *deltas) //NOT TESTED
{
    for (unsigned int i=0; i<numberOfJoints; ++i) {
        referencePosition[i] = pos[i]+ deltas[i]; //TODO check if this is ok or referencePosition=referencePosition+delta!!!
    }
    return true;
}

bool GazeboYarpControlBoardDriver::checkMotionDone(int j, bool *flag) //NOT TESTED
{
    if (flag && j >= 0 && j < (int)numberOfJoints) {
        *flag = motion_done[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::checkMotionDone(bool *flag) //NOT TESTED
{
    if (!flag) return false;
    bool temp_flag = true;
    //*flag=true;
    for(unsigned int j = 0; j < numberOfJoints; ++j)
    {
        temp_flag = temp_flag && motion_done[j];
    }
    *flag = temp_flag;
    return true;
}

bool GazeboYarpControlBoardDriver::setPositionMode() //NOT TESTED
{
    for (unsigned int j=0; j<numberOfJoints; j++)
    {
        this->setPositionMode(j);
    }
    return true;
}

bool GazeboYarpControlBoardDriver::setRefSpeeds(const double *spds) //NOT TESTED
{
    for (unsigned int i=0; i<numberOfJoints; ++i) {
        referenceSpeed[i] = spds[i];
    }
    return true;
}


bool GazeboYarpControlBoardDriver::setRefAcceleration(int j, double acc) //NOT IMPLEMENTED
{
    if (j >= 0 && j < (int)numberOfJoints) {
        referenceAcceleraton[j] = acc;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::setRefAccelerations(const double *accs) //NOT IMPLEMENTED
{
    for (unsigned int i=0; i<numberOfJoints; ++i) {
        referenceAcceleraton[i] = accs[i];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getRefAcceleration(int j, double *acc) //NOT IMPLEMENTED
{
    if (acc && j >= 0 && j < (int)numberOfJoints) {
        *acc = referenceAcceleraton[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getRefAccelerations(double *accs) //NOT IMPLEMENTED
{
    if (!accs) return false;
    for (unsigned int i=0; i<numberOfJoints; ++i) {
        accs[i] = referenceAcceleraton[i];
    }
    return true;
}

// IPositionControl2

bool GazeboYarpControlBoardDriver::positionMove(const int n_joint, const int *joints, const double *refs) //NOT IMPLEMENTED
{
    bool ret = true;
    for(int i=0; i<n_joint; i++)
    {
        ret = ret && positionMove(joints[i], refs[i]);
    }
    return ret;
}

bool GazeboYarpControlBoardDriver::relativeMove(const int n_joint, const int *joints, const double *deltas) //NOT IMPLEMENTED
{
    return false;
}


bool GazeboYarpControlBoardDriver::checkMotionDone(const int n_joint, const int *joints, bool *flags) //NOT IMPLEMENTED
{
    return false;
}

bool GazeboYarpControlBoardDriver::setRefSpeeds(const int n_joint, const int *joints, const double *spds) //NOT IMPLEMENTED
{
    return false;
}


bool GazeboYarpControlBoardDriver::setRefAccelerations(const int n_joint, const int *joints, const double *accs) //NOT IMPLEMENTED
{
    return false;
}


bool GazeboYarpControlBoardDriver::getRefSpeeds(const int n_joint, const int *joints, double *spds) //NOT IMPLEMENTED
{
    return false;
}


bool GazeboYarpControlBoardDriver::getRefAccelerations(const int n_joint, const int *joints, double *accs) //NOT IMPLEMENTED
{
    return false;
}


bool GazeboYarpControlBoardDriver::stop(const int n_joint, const int *joints) //NOT IMPLEMENTED
{
    return false;
}


// IPOSITION DIRECT
bool GazeboYarpControlBoardDriver::setPositionDirectMode() //NOT IMPLEMENTED -> Is it the same as setPositionMode?
{
    return false;
}

bool GazeboYarpControlBoardDriver::setPosition(int j, double ref)
{
    if (j >= 0 && j < (int)numberOfJoints)
    {
        desiredPosition[j] = ref;
        return positionMove(j, ref);
    }
    return false;
}

bool GazeboYarpControlBoardDriver::setPositions(const int n_joint, const int *joints, double *refs)
{
    for (unsigned int i = 0; i < numberOfJoints; ++i)
        desiredPosition[i] = refs[i];
    return positionMove(n_joint, joints, refs);
}

bool GazeboYarpControlBoardDriver::setPositions(const double *refs)
{
    return positionMove(refs);
}
