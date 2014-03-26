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
    if (j >= 0 && j < (int)_controlboard_number_of_joints) {
        ref_pos[j] = ref; //we will use this ref_pos in the next simulation onUpdate call to ask gazebo to set PIDs ref_pos to this value
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::stop(int j) //WORKS
{
    if (j >= 0 && j < (int)_controlboard_number_of_joints) {
        ref_pos[j] = pos[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::stop() //WORKS
{
    ref_pos=pos;
    return true;
}

bool GazeboYarpControlBoardDriver::positionMove(const double *refs) //WORKS
{
    for (unsigned int i = 0; i < _controlboard_number_of_joints; ++i) {
        ref_pos[i] = refs[i];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getAxes(int *ax) // WORKS
{
    if (!ax) return false;
    *ax = _controlboard_number_of_joints;
    return true;
}

bool GazeboYarpControlBoardDriver::setRefSpeed(int j, double sp) //WORKS
{
    if (j >= 0 && j < (int)_controlboard_number_of_joints) {
        ref_speed[j] = sp;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getRefSpeed(int j, double *ref) //WORKS
{
    if (ref && j >= 0 && j < (int)_controlboard_number_of_joints) {
        *ref = ref_speed[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getRefSpeeds(double *spds) //WORKS
{
    if (!spds) return false;
    for (unsigned int i = 0; i < _controlboard_number_of_joints; ++i) {
        spds[i] = ref_speed[i];
    }
    return true;
}



bool GazeboYarpControlBoardDriver::relativeMove(int j, double delta) //NOT TESTED
{
    if (j >= 0 && j < (int)_controlboard_number_of_joints) {
        ref_pos[j] = pos[j] + delta; //TODO check if this is ok or ref_pos=ref_pos+delta!!!
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::relativeMove(const double *deltas) //NOT TESTED
{
    for (unsigned int i=0; i<_controlboard_number_of_joints; ++i) {
        ref_pos[i] = pos[i]+ deltas[i]; //TODO check if this is ok or ref_pos=ref_pos+delta!!!
    }
    return true;
}

bool GazeboYarpControlBoardDriver::checkMotionDone(int j, bool *flag) //NOT TESTED
{
    if (flag && j >= 0 && j < (int)_controlboard_number_of_joints) {
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
    for(unsigned int j = 0; j < _controlboard_number_of_joints; ++j)
    {
        temp_flag = temp_flag && motion_done[j];
    }
    *flag = temp_flag;
    return true;
}

bool GazeboYarpControlBoardDriver::setPositionMode() //NOT TESTED
{
    for (unsigned int j=0; j<_controlboard_number_of_joints; j++)
    {
        this->setPositionMode(j);
    }
    return true;
}

bool GazeboYarpControlBoardDriver::setRefSpeeds(const double *spds) //NOT TESTED
{
    for (unsigned int i=0; i<_controlboard_number_of_joints; ++i) {
        ref_speed[i] = spds[i];
    }
    return true;
}


bool GazeboYarpControlBoardDriver::setRefAcceleration(int j, double acc) //NOT IMPLEMENTED
{
    if (j >= 0 && j < (int)_controlboard_number_of_joints) {
        ref_acc[j] = acc;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::setRefAccelerations(const double *accs) //NOT IMPLEMENTED
{
    for (unsigned int i=0; i<_controlboard_number_of_joints; ++i) {
        ref_acc[i] = accs[i];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getRefAcceleration(int j, double *acc) //NOT IMPLEMENTED
{
    if (acc && j >= 0 && j < (int)_controlboard_number_of_joints) {
        *acc = ref_acc[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getRefAccelerations(double *accs) //NOT IMPLEMENTED
{
    if (!accs) return false;
    for (unsigned int i=0; i<_controlboard_number_of_joints; ++i) {
        accs[i] = ref_acc[i];
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
bool GazeboYarpControlBoardDriver::setPosition(int j, double ref)
{
    if (j >= 0 && j < (int)_controlboard_number_of_joints)
    {
        des_pos[j] = ref;
        return positionMove(j, ref);
    }
    return false;
}

bool GazeboYarpControlBoardDriver::setPositions(const int n_joint, const int *joints, double *refs)
{
    for (unsigned int i = 0; i < _controlboard_number_of_joints; ++i)
        des_pos[i] = refs[i];
    return positionMove(n_joint, joints, refs);
}

bool GazeboYarpControlBoardDriver::setPositions(const double *refs)
{
    return positionMove(refs);
}
