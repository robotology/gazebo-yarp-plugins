#include "coman.h"


using namespace yarp::dev;


bool coman::getEncoder(int j, double *v) //WORKS
{
    //pos_lock.lock();
    if (j<_robot_number_of_joints) {
        (*v) = pos[j]-zero_pos[j];
    }
    //pos_lock.unlock();
    return true;
    
}

bool coman::getEncoders(double *encs) //WORKS
{
    //pos_lock.lock();
    for (int i=0; i<_robot_number_of_joints; ++i) {
        encs[i] = pos[i]-zero_pos[i];  //should we just use memcopy here?
    }
    return true;
    //pos_lock.unlock();
}



/**
 * Since we don't know how to reset gazebo encoders, we will simply add the actual value to the future encoders readings
 */
bool coman::resetEncoder(int j) //WORKS
{
    if (j<_robot_number_of_joints) {
        zero_pos[j] = pos[j];
    }
    return true;
}

bool coman::resetEncoders() //WORKS
{
    for (int i=0; i<_robot_number_of_joints; ++i) {
        zero_pos[i] = pos[i];
    }
    return true;
}

bool coman::setEncoder(int j, double val) //WORKS
{
    if (j<_robot_number_of_joints) {
        zero_pos[j] = pos[j]-val;
    }
    return true;
}

bool coman::setEncoders(const double *vals) //WORKS
{
    for (int i=0; i<_robot_number_of_joints; ++i) {
        zero_pos[i] = pos[i]-vals[i];
    }
    return true;
}


bool coman::getEncoderSpeed(int j, double *sp) //NOT TESTED
{
    if ( j < _robot_number_of_joints) {
        (*sp) = speed[j];
    }
    return true;
}

bool coman::getEncoderSpeeds(double *spds) //NOT TESTED
{
    for (unsigned int i = 0; i < _robot_number_of_joints; ++i) {
        getEncoderSpeed(i, spds);
    }
    return true;
}

bool coman::getEncoderAcceleration(int j, double *spds) //NOT IMPLEMENTED
{
    if (j<_robot_number_of_joints) {
        (*spds) = 0;
    }
    return true;
}

bool coman::getEncoderAccelerations(double *accs) //NOT IMPLEMENTED
{
    for (int i=0; i<_robot_number_of_joints; ++i) {
        accs[i] = 0;
    }
    return true;
}

