/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "ControlBoardDriver.h"


using namespace yarp::dev;


bool GazeboYarpControlBoardDriver::setVelocityMode() //NOT TESTED
{
    bool ret = true;
    for (unsigned int j = 0; j < m_numberOfJoints; j++) {
        ret = ret && this->setControlMode(j, VOCAB_CM_VELOCITY);
    }
    return ret;
}

bool GazeboYarpControlBoardDriver::velocityMove(int j, double sp) //NOT TESTED
{
    if (j >= 0 && j < (int)m_numberOfJoints)
    {
        m_jntReferenceVelocities[j] = sp;
        m_velocity_watchdog[j]->reset();
        if (m_speed_ramp_handler[j])
          {  m_speed_ramp_handler[j]->setReference(m_jntReferenceVelocities[j], m_trajectoryGenerationReferenceAcceleration[j]); }
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::velocityMove(const double *sp) //NOT TESTED
{
    if (!sp) return false;
    for (unsigned int i = 0; i < m_numberOfJoints; ++i)
    {
        velocityMove(i, sp[i]);
    }
    return true;
}

bool GazeboYarpControlBoardDriver::velocityMove(const int n_joint, const int *joints, const double *spds)
{
    if (!joints || !spds) return false;
    bool ret = true;
    for (int i = 0; i < n_joint && ret; i++)
    {
        ret = velocityMove(joints[i], spds[i]);
    }
    return ret;
}

bool GazeboYarpControlBoardDriver::setVelPid(int j, const yarp::dev::Pid &pid)
{
    if (j >= 0 && j < (int)m_numberOfJoints)
    {
        // Converting all gains for degrees-based unit to radians-based
        m_velocityPIDs[j].p = convertUserGainToGazeboGain(j, pid.kp);
        m_velocityPIDs[j].i = convertUserGainToGazeboGain(j, pid.ki);
        m_velocityPIDs[j].d = convertUserGainToGazeboGain(j, pid.kd);
        // The output limits are only related to the output, so they don't need to be converted
        m_velocityPIDs[j].maxInt = pid.max_int;
        m_velocityPIDs[j].maxOut = pid.max_output;    
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::setVelPids(const yarp::dev::Pid *pids)
{
    if (!pids) return false;
    bool b = true;
    for (unsigned j = 0; j < m_numberOfJoints; j++)
    {
        b &=setVelPid (j,pids[j]);
    }
    return b;
}

bool GazeboYarpControlBoardDriver::getVelPid(int j, yarp::dev::Pid *pid)
{
    if (!pid) return false;
    if (j >= 0 && j < (int)m_numberOfJoints)
    {
      // Converting all gains for degrees-based unit to radians-based
      pid->kp = convertGazeboGainToUserGain(j, m_velocityPIDs[j].p);
      pid->ki = convertGazeboGainToUserGain(j, m_velocityPIDs[j].i);
      pid->kd = convertGazeboGainToUserGain(j, m_velocityPIDs[j].d);

      // The output limits are only related to the output, so they don't need to be converted
      pid->max_int = m_velocityPIDs[j].maxInt;
      pid->max_output = m_velocityPIDs[j].maxOut;
      return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getVelPids(yarp::dev::Pid *pids)
{
    if (!pids) return false;
    bool b = true;
    for (unsigned j = 0; j < m_numberOfJoints; j++)
    {
        b &=getVelPid (j,&pids[j]);
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getRefVelocity(const int joint, double *vel) 
{
    if (vel && joint >= 0 && joint < (int)m_numberOfJoints)
    {
      *vel = m_jntReferenceVelocities[joint];
      return true;
    }
    return false;
  
}

bool GazeboYarpControlBoardDriver::getRefVelocities(double *vels) 
{
    if (!vels) return false; //check or not check?
    bool ret = true;
    for (int i = 0; i < this->m_numberOfJoints && ret; i++) {
        ret = getRefVelocity(i, &vels[i]);
    }
    return ret; 
}

bool GazeboYarpControlBoardDriver::getRefVelocities(const int n_joint, const int *joints, double *vels) 
{
    if (!joints || !vels) return false; //check or not check?
    bool ret = true;
    for (int i = 0; i < n_joint && ret; i++) {
        ret = getRefVelocity(joints[i], &vels[i]);
    }
    return ret;
}
