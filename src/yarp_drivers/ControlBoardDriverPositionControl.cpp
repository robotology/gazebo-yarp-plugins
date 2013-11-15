/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */



#include <gazebo_yarp_plugins/ControlBoardDriver.h>


using namespace yarp::dev;


/**
 * This is asyncronous, but do we care?
 */
bool GazeboYarpControlBoardDriver::positionMove(int j, double ref) //WORKS
{
    if (j<_robot_number_of_joints) {
        ref_pos[j] = ref; //we will use this ref_pos in the next simulation onUpdate call to ask gazebo to set PIDs ref_pos to this value
    }
    return true;
}

bool GazeboYarpControlBoardDriver::stop(int j) //WORKS
{
    ref_pos[j]=pos[j];
    return true;
}

bool GazeboYarpControlBoardDriver::stop() //WORKS
{
    ref_pos=pos;
    return true;
}

bool GazeboYarpControlBoardDriver::positionMove(const double *refs) //WORKS
{
    for (unsigned int i=0; i<_robot_number_of_joints; ++i) {
        ref_pos[i] = refs[i];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getAxes(int *ax) // WORKS
{
    *ax = _robot_number_of_joints;
    return true;
}

bool GazeboYarpControlBoardDriver::setRefSpeed(int j, double sp) //WORKS
{
    if (j<_robot_number_of_joints) {
        ref_speed[j] = sp;
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getRefSpeed(int j, double *ref) //WORKS
{
    if (j<_robot_number_of_joints) {
        (*ref) = ref_speed[j];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getRefSpeeds(double *spds) //WORKS
{
    for (unsigned int i=0; i<_robot_number_of_joints; ++i) {
        spds[i] = ref_speed[i];
    }
    return true;
}



bool GazeboYarpControlBoardDriver::relativeMove(int j, double delta) //NOT TESTED
{
    if (j<_robot_number_of_joints) {
        ref_pos[j] =pos[j] + delta; //TODO check if this is ok or ref_pos=ref_pos+delta!!!
    }
    return true;
}

bool GazeboYarpControlBoardDriver::relativeMove(const double *deltas) //NOT TESTED
{
    for (unsigned int i=0; i<_robot_number_of_joints; ++i) {
        ref_pos[i] = pos[i]+ deltas[i]; //TODO check if this is ok or ref_pos=ref_pos+delta!!!
    }
    return true;
}

bool GazeboYarpControlBoardDriver::checkMotionDone(int j, bool *flag) //NOT TESTED
{
    *flag=motion_done[j];
    return true;
}

bool GazeboYarpControlBoardDriver::checkMotionDone(bool *flag) //NOT TESTED
{
    bool temp_flag=true;
    //*flag=true;
    for(unsigned int j=0; j<_robot_number_of_joints; ++j)
    {
        //*flag&&motion_done[j]; //It's compiler job to make code unreadable and optimized, not programmer's
        temp_flag=temp_flag && motion_done[j];
    }
    *flag=temp_flag;
    return true;
}

bool GazeboYarpControlBoardDriver::setPositionMode() //NOT TESTED
{
    for(unsigned int j=0; j<_robot_number_of_joints; j++)
    {
        this->setPositionMode(j);
    }
    return true;
}

bool GazeboYarpControlBoardDriver::setRefSpeeds(const double *spds) //NOT TESTED
{
    for (unsigned int i=0; i<_robot_number_of_joints; ++i) {
        ref_speed[i] = spds[i];
    }
    return true;
}




bool GazeboYarpControlBoardDriver::setRefAcceleration(int j, double acc) //NOT IMPLEMENTED
{
    if (j<_robot_number_of_joints) {
        ref_acc[j] = acc;
    }
    return true;
}

bool GazeboYarpControlBoardDriver::setRefAccelerations(const double *accs) //NOT IMPLEMENTED
{
    for (unsigned int i=0; i<_robot_number_of_joints; ++i) {
        ref_acc[i] = accs[i];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getRefAcceleration(int j, double *acc) //NOT IMPLEMENTED
{
    if (j<_robot_number_of_joints) {
        (*acc) = ref_acc[j];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getRefAccelerations(double *accs) //NOT IMPLEMENTED
{
    for (unsigned int i=0; i<_robot_number_of_joints; ++i) {
        accs[i] = ref_acc[i];
    }
    return true;
}