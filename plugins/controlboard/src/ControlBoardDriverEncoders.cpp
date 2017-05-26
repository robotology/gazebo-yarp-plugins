/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "ControlBoardDriver.h"

using namespace yarp::dev;


bool GazeboYarpControlBoardDriver::getEncoder(int j, double *v) //WORKS
{
    if (v && j >= 0 && static_cast<size_t>(j) < m_numberOfJoints) {
        *v = m_positions[j]-m_zeroPosition[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getEncoders(double *encs) //WORKS
{
    if (!encs) return false;
    for (size_t i = 0; i < m_numberOfJoints; ++i) {
        encs[i] = m_positions[i]-m_zeroPosition[i];  //should we just use memcopy here?
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getEncodersTimed(double *encs, double *time)
{
    double my_time = m_lastTimestamp.getTime();
    for (size_t i = 0; i < m_numberOfJoints; ++i) {
        encs[i] = m_positions[i]-m_zeroPosition[i];  //should we just use memcopy here?
        time[i] = my_time;
    }

    return true;
}

/**
 * Read the instantaneous acceleration of the specified axis
 * @param j axis index
 * @param encs pointer to double
 * @param time corresponding timestamp (pointer to)
 * @return true if all goes well, false if anything bad happens.
 */
bool GazeboYarpControlBoardDriver::getEncoderTimed(int j, double *encs, double *time)
{
    if (time && encs && j >= 0 && static_cast<size_t>(j) < m_numberOfJoints) {
        *encs = m_positions[j]-m_zeroPosition[j];
        *time = m_lastTimestamp.getTime();
        return true;
    }
    return false;
}


/**
 * Since we don't know how to reset gazebo encoders, we will simply add the actual value to the future encoders readings
 */
bool GazeboYarpControlBoardDriver::resetEncoder(int j) //WORKS
{
    if (j >= 0 && static_cast<size_t>(j) < m_numberOfJoints) {
        m_zeroPosition[j] = m_positions[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::resetEncoders() //WORKS
{
    for (size_t i = 0; i < m_numberOfJoints; ++i) {
        m_zeroPosition[i] = m_positions[i];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::setEncoder(int j, double val) //WORKS
{
    if (j >= 0 && static_cast<size_t>(j) < m_numberOfJoints) {
        m_zeroPosition[j] = m_positions[j]-val;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::setEncoders(const double *vals) //WORKS
{
    for (size_t i=0; i < m_numberOfJoints; ++i) {
        m_zeroPosition[i] = m_positions[i]-vals[i];
    }
    return true;
}


bool GazeboYarpControlBoardDriver::getEncoderSpeed(int j, double *sp) //NOT TESTED
{
    if (sp && j >= 0 && static_cast<size_t>(j) < m_numberOfJoints) {
        *sp = m_velocities[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getEncoderSpeeds(double *spds) //NOT TESTED
{
    if (!spds) return false;
    for (size_t i = 0; i < m_numberOfJoints; ++i) {
        getEncoderSpeed(i, &spds[i]);
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getEncoderAcceleration(int j, double *spds) //NOT IMPLEMENTED
{
    if (spds && j >= 0 && static_cast<size_t>(j) < m_numberOfJoints) {
        *spds = 0.0;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getEncoderAccelerations(double *accs) //NOT IMPLEMENTED
{
    if (!accs) return false;
    for (size_t i = 0; i < m_numberOfJoints; ++i) {
        accs[i] = 0.0;
    }
    return true;
}
