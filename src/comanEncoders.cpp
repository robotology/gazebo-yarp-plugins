/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include "coman.h"


using namespace yarp::dev;


bool GazeboYarpControlBoardDriver::getEncoder(int j, double *v) //WORKS
{
    //pos_lock.lock();
    if (j<_robot_number_of_joints) {
        (*v) = pos[j]-zero_pos[j];
    }
    //pos_lock.unlock();
    return true;
}

bool GazeboYarpControlBoardDriver::getEncoders(double *encs) //WORKS
{
    //pos_lock.lock();
    for (unsigned int i=0; i<_robot_number_of_joints; ++i) {
        encs[i] = pos[i]-zero_pos[i];  //should we just use memcopy here?
    }
    return true;
    //pos_lock.unlock();
}

/**
 * Since we don't know how to reset gazebo encoders, we will simply add the actual value to the future encoders readings
 */
bool GazeboYarpControlBoardDriver::resetEncoder(int j) //WORKS
{
    if (j<_robot_number_of_joints) {
        zero_pos[j] = pos[j];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::resetEncoders() //WORKS
{
    for (unsigned int i=0; i<_robot_number_of_joints; ++i) {
        zero_pos[i] = pos[i];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::setEncoder(int j, double val) //WORKS
{
    if (j<_robot_number_of_joints) {
        zero_pos[j] = pos[j]-val;
    }
    return true;
}

bool GazeboYarpControlBoardDriver::setEncoders(const double *vals) //WORKS
{
    for (unsigned int i=0; i<_robot_number_of_joints; ++i) {
        zero_pos[i] = pos[i]-vals[i];
    }
    return true;
}





bool GazeboYarpControlBoardDriver::getEncoderSpeed(int j, double *sp) //NOT TESTED
{
    if ( j < _robot_number_of_joints) {
        (*sp) = speed[j];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getEncoderSpeeds(double *spds) //NOT TESTED
{
    for (unsigned int i = 0; i < _robot_number_of_joints; ++i) {
        getEncoderSpeed(i, spds);
    }
    return true;
}




bool GazeboYarpControlBoardDriver::getEncoderAcceleration(int j, double *spds) //NOT IMPLEMENTED
{
    if (j<_robot_number_of_joints) {
        (*spds) = 0;
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getEncoderAccelerations(double *accs) //NOT IMPLEMENTED
{
    for (unsigned int i=0; i<_robot_number_of_joints; ++i) {
        accs[i] = 0;
    }
    return true;
}

