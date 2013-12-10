/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */



#include <gazebo_yarp_plugins/ControlBoardDriver.h>


using namespace yarp::dev;


bool GazeboYarpControlBoardDriver::getEncoder(int j, double *v) //WORKS
{
    if (v && j >= 0 && j < (int)_robot_number_of_joints) {
        *v = pos[j]-zero_pos[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getEncoders(double *encs) //WORKS
{
    if (!encs) return false;
    for (unsigned int i = 0; i < _robot_number_of_joints; ++i) {
        encs[i] = pos[i]-zero_pos[i];  //should we just use memcopy here?
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getEncodersTimed(double *encs, double *time)
{
    double my_time = yarp::os::Time::now();
    for (unsigned int i = 0; i <_robot_number_of_joints; ++i) {
        encs[i] = pos[i]-zero_pos[i];  //should we just use memcopy here?
        time[i] = my_time;
    }
    
    return true;
}

/**
 * Read the instantaneous acceleration of all axes.
 * @j axis index
 * @enc encoder value (pointer to)
 * @stamp corresponding timestamp (pointer to)
 * @return true if all goes well, false if anything bad happens.
 */
bool GazeboYarpControlBoardDriver::getEncoderTimed(int j, double *encs, double *time)
{
    if (time && encs && j >= 0 && j < (int)_robot_number_of_joints) {
        *encs = pos[j]-zero_pos[j];
        *time = yarp::os::Time::now();
        return true;
    }
    return false;
}


/**
 * Since we don't know how to reset gazebo encoders, we will simply add the actual value to the future encoders readings
 */
bool GazeboYarpControlBoardDriver::resetEncoder(int j) //WORKS
{
    if (j >= 0 && j < (int)_robot_number_of_joints) {
        zero_pos[j] = pos[j];
        return true;
    }
    return false;
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
    if (j >= 0 && j < (int)_robot_number_of_joints) {
        zero_pos[j] = pos[j]-val;
        return true;
    }
    return false;
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
    if (sp && j >= 0 && j < (int)_robot_number_of_joints) {
        *sp = speed[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getEncoderSpeeds(double *spds) //NOT TESTED
{
    if (!spds) return false;
    for (unsigned int i = 0; i < _robot_number_of_joints; ++i) {
        getEncoderSpeed(i, &spds[i]);
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getEncoderAcceleration(int j, double *spds) //NOT IMPLEMENTED
{
    if (spds && j >= 0 && j < (int)_robot_number_of_joints) {
        *spds = 0;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getEncoderAccelerations(double *accs) //NOT IMPLEMENTED
{
    if (!accs) return false;
    for (unsigned int i=0; i<_robot_number_of_joints; ++i) {
        accs[i] = 0;
    }
    return true;
}
