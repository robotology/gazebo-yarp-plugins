/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: Marco Randazzo <marco.randazzo@iit.it>
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "ControlBoardDriver.h"
#include <GazeboYarpPlugins/common.h>

#include <ControlBoardDriverTrajectory.h>
#include <cstdio>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/math/Angle.hh>

#include <yarp/os/LogStream.h>
#include <yarp/sig/Image.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

void Watchdog::reset()
{
    m_lastUpdate=yarp::os::Time::now();
}

bool Watchdog::isExpired()
{
    if (m_duration<0) return false;
    if ((yarp::os::Time::now()- m_lastUpdate) > m_duration) return true;
    return false;
}

Watchdog::Watchdog (double expireTime)
{
    m_duration=expireTime;
    m_lastUpdate=yarp::os::Time::now();
}

void Watchdog::modifyDuration(double expireTime)
{
     m_duration=expireTime; 
}

double Watchdog::getDuration()
{
     return m_duration; 
}


RampFilter::RampFilter()
{
    m_final_reference = 0;
    m_current_value = 0;
    m_step = 0;
}

void RampFilter::setReference(double ref, double step)
{
    m_mutex.wait();
    m_final_reference = ref;
    m_step = step;
    m_mutex.post();
}

void RampFilter::stop()
{
    m_mutex.wait();
    m_final_reference = 0.0;
    m_current_value = 0.0;
    m_step = 0.0;
    m_mutex.post();
}

void RampFilter::update()
{
    m_mutex.wait();
    double tmp = 0;
    double error_abs = fabs(m_final_reference - m_current_value);

    if (m_final_reference > m_current_value)
    {
        tmp = +(m_step / 1000.0) * 1.0; //* m_controllerPeriod* _T_controller;
    }
    else
    {
        tmp = -(m_step / 1000.0) * 1.0; //* m_controllerPeriod*_T_controller;
    }

    if( error_abs < fabs(tmp) )
    {
        tmp = (m_final_reference - m_current_value);
    }

    m_current_value = m_current_value + tmp;
    m_mutex.post();
}

double RampFilter::getCurrentValue()
{
    return m_current_value;
}

//------------------------------------------------------------------------------------------------------------------
// TrajectoryGenerator
//------------------------------------------------------------------------------------------------------------------

TrajectoryGenerator::TrajectoryGenerator(gazebo::physics::Model* model)
: m_robot(model)
, m_trajectory_complete(true)
, m_speed(0)
, m_joint_min(0)
, m_joint_max(0)
{}

TrajectoryGenerator::~TrajectoryGenerator() {}

bool TrajectoryGenerator::isMotionDone()
{
  return m_trajectory_complete;
}

bool  TrajectoryGenerator::setLimits(double min, double max)
{
  m_joint_min = min;
  m_joint_max = max;
  return true;
}

//------------------------------------------------------------------------------------------------------------------
// MinJerkTrajectoryGenerator
//------------------------------------------------------------------------------------------------------------------

MinJerkTrajectoryGenerator::MinJerkTrajectoryGenerator(gazebo::physics::Model* model)
: TrajectoryGenerator(model) {}

MinJerkTrajectoryGenerator::~MinJerkTrajectoryGenerator() {}

double MinJerkTrajectoryGenerator::p_compute_p5f (double t)
{
    double accum = m_dx0*t;
    double tmp = t * t * t;
    accum = accum + tmp * m_trajectory_coeff_c1;//(10*xfx0 - 6*dx0);
    tmp *= t;
    accum = accum - tmp * m_trajectory_coeff_c2;//(15*xfx0 - 8*dx0);
    tmp *= t;
    accum = accum + tmp * m_trajectory_coeff_c3;//(6*xfx0 - 3*dx0);
    return accum;
}

double MinJerkTrajectoryGenerator::p_compute_p5f_vel (double t)
{
    double accum = -2*m_dx0*t - m_dx0;
    accum = accum + (30*m_x0 +15*m_dx0    -30*m_xf) * (t*t);
    accum = -accum/m_tf * (t-1)*(t-1);
    return accum;
}

double MinJerkTrajectoryGenerator::p_compute_current_vel()
{
    double a;

    //(10 * (t/T)^3 - 15 * (t/T)^4 + 6 * (t/T)^5) * (x0-xf) + x0
    if (m_trajectory_complete) return 0;
    if (m_cur_t == 0)
    {
        return 0;
    }
    else  if (m_cur_t < 1.0 - m_step)
    {
        // calculate the velocity
        a = p_compute_p5f_vel (m_cur_t);
        return a;
    }
    return 0;
}

bool MinJerkTrajectoryGenerator::p_abortTrajectory (double limit)
{
    if (!m_trajectory_complete)
    {
        m_trajectory_complete = true;
        m_cur_t = 0;
        m_cur_step = 0;
        m_xf = limit;
    }
    else
    {
        m_cur_t = 0;
        m_cur_step = 0;
        m_xf = limit;
    }
    return true;
}

bool MinJerkTrajectoryGenerator::abortTrajectory (double limit)
{
    m_mutex.wait();
    bool ret = p_abortTrajectory(limit);
    m_mutex.post();
    return ret;
}

bool MinJerkTrajectoryGenerator::initTrajectory (double current_pos, double final_pos, double speed)
{
    m_mutex.wait();
    m_controllerPeriod = (unsigned)(this->m_robot->GetWorld()->GetPhysicsEngine()->GetUpdatePeriod() * 1000.0);
    double speedf = fabs(speed);
    double dx0 =0;
    m_computed_reference = current_pos;

    if (speed <= 0)
    {
      m_mutex.post();
      return false; 
    }

    m_dx0 = p_compute_current_vel();
    m_x0 = current_pos;
    m_prev_a = current_pos;
    m_xf = final_pos;
    if (m_xf > m_joint_max) m_xf = m_joint_max;
    else if(m_xf < m_joint_min) m_xf = m_joint_min;
    m_speed = speed;

    //double step = (m_trajectoryGenerationReferenceSpeed[j] / 1000.0) * m_robotRefreshPeriod * _T_controller;

    m_tf = (1000 * fabs(m_xf - m_x0) / speedf) / double (m_controllerPeriod);
    m_dx0 = m_dx0 * m_tf;

    dx0  = m_dx0;
    m_trajectory_coeff_c1 = (10*(m_xf - m_x0) - 6*dx0);
    m_trajectory_coeff_c2 = (15*(m_xf - m_x0) - 8*dx0);
    m_trajectory_coeff_c3 = (6* (m_xf - m_x0) - 3*dx0);

    if (m_tf < 1 || m_tf == 0)
    {
        p_abortTrajectory (final_pos);
        m_step=0;
        m_mutex.post();
        return false;
    }
    else
    {
        m_step = 1 / m_tf;
    }
    m_cur_t = 0;
    m_cur_step = 0;
    m_trajectory_complete =  false;
    
    m_mutex.post();
    return true;
}

double MinJerkTrajectoryGenerator::p_computeTrajectoryStep()
{
    double target=0;
    double delta_target=0;

    // (10 * (t/T)^3 - 15 * (t/T)^4 + 6 * (t/T)^5) * (x0-xf) + x0
    if (m_trajectory_complete)
    {
        target = m_xf;
        delta_target = target - m_prev_a;
        m_prev_a = target;
        //yDebug()<<"mdone" << target;
        return delta_target;
    }

    if (m_cur_t== 0)
    {
        m_cur_t += m_step;
        m_cur_step ++;

        target = m_x0;
        delta_target = target - m_prev_a;
        m_prev_a = target;
        //yDebug()<<"first" ;
        return delta_target;
    }
    else if (m_cur_t < 1.0 - m_step)
    {
        // calculate the power factors
        target = p_compute_p5f (m_cur_t);
        target += m_x0;

        // time
        m_cur_t += m_step;
        m_cur_step ++;

        delta_target = target - m_prev_a;
        m_prev_a = target;
        //yDebug()<< delta_target;
        return delta_target;
    }

    //yDebug()<<"last";
    m_trajectory_complete = true;
    target =m_xf;

    //position reached, so step is zero
    return 0.0;
}

double MinJerkTrajectoryGenerator::computeTrajectoryStep()
{
    m_mutex.wait();
    double ret = p_computeTrajectoryStep();
    m_mutex.post();
    return ret;
}

double MinJerkTrajectoryGenerator::p_computeTrajectory()
{
    double target=0;

    // (10 * (t/T)^3 - 15 * (t/T)^4 + 6 * (t/T)^5) * (x0-xf) + x0
    if (m_trajectory_complete)
    {
        target = m_xf;
        m_prev_a = target;
        //yDebug()<<"mdone" << target;
        return target;
    }

    if (m_cur_t== 0)
    {
        m_cur_t += m_step;
        m_cur_step ++;

        target = m_x0;
        m_prev_a = target;
        //yDebug()<<"first" ;
        return target;
    }
    else if (m_cur_t < 1.0 - m_step)
    {
        // calculate the power factors
        target = p_compute_p5f (m_cur_t);
        target += m_x0;

        // time
        m_cur_t += m_step;
        m_cur_step ++;

        m_prev_a = target;
        //yDebug()<< target;
        return target;
    }

    //yDebug()<<"last";
    m_trajectory_complete = true;
    target =m_xf;
    return target;  
}

double MinJerkTrajectoryGenerator::computeTrajectory()
{   
    m_mutex.wait();
    double ret = p_computeTrajectory();
    m_mutex.post();
    return ret;
}

yarp::dev::TrajectoryType MinJerkTrajectoryGenerator::getTrajectoryType()
{
    return yarp::dev::TRAJECTORY_TYPE_MIN_JERK;
}


//------------------------------------------------------------------------------------------------------------------
// ConstSpeedTrajectoryGenerator
//------------------------------------------------------------------------------------------------------------------

ConstSpeedTrajectoryGenerator::ConstSpeedTrajectoryGenerator(gazebo::physics::Model* model)
: TrajectoryGenerator(model) {}

ConstSpeedTrajectoryGenerator::~ConstSpeedTrajectoryGenerator() {}

bool ConstSpeedTrajectoryGenerator::initTrajectory (double current_pos, double final_pos, double speed)
{
    m_mutex.wait();
    m_controllerPeriod = (unsigned)(this->m_robot->GetWorld()->GetPhysicsEngine()->GetUpdatePeriod() * 1000.0);
    m_x0 = current_pos;
    m_xf = final_pos;
    if (m_xf > m_joint_max) m_xf = m_joint_max;
    else if(m_xf < m_joint_min) m_xf = m_joint_min;
    m_speed = speed;
    m_computed_reference = m_x0;
    m_mutex.post();
    return true;
}

bool ConstSpeedTrajectoryGenerator::p_abortTrajectory(double limit)
{
    //to be implemented
  return true;
}

bool ConstSpeedTrajectoryGenerator::abortTrajectory(double limit)
{
    m_mutex.wait();
    bool ret = p_abortTrajectory(limit);
    m_mutex.post();
    return ret;
}

double ConstSpeedTrajectoryGenerator::p_computeTrajectoryStep()
{
    double step = 0;
    double error_abs = fabs(m_xf - m_computed_reference);

    if (m_xf > m_computed_reference)
    {
        step = +(m_speed / 1000.0) * m_controllerPeriod; //* _T_controller;
    }
    else
    {
        step = -(m_speed / 1000.0) * m_controllerPeriod; //* _T_controller;
    }

    if( error_abs < fabs(step) )
    {
        step = (m_xf - m_computed_reference);
    }

    //yDebug() << step;
    return step;
}

double ConstSpeedTrajectoryGenerator::computeTrajectoryStep()
{
    m_mutex.wait();
    double ret = p_computeTrajectoryStep();
    m_mutex.post();
    return ret;
}

double ConstSpeedTrajectoryGenerator::computeTrajectory()
{
    m_mutex.wait();
    double ret = p_computeTrajectory();
    m_mutex.post();
    return ret;
}

double ConstSpeedTrajectoryGenerator::p_computeTrajectory()
{
    double step = (m_speed / 1000.0) * m_controllerPeriod; //* _T_controller;
    double error_abs = fabs(m_computed_reference - m_xf);
    
    if(error_abs)
    {
        m_computed_reference += p_computeTrajectoryStep(); 
        if( error_abs < step)  
        {
            m_trajectory_complete = true;
        }
        else
        {
            m_trajectory_complete = false;
        }
    }
    
    //yDebug() << m_computed_reference;
    return m_computed_reference;
}

yarp::dev::TrajectoryType ConstSpeedTrajectoryGenerator::getTrajectoryType()
{
    return yarp::dev::TRAJECTORY_TYPE_CONST_SPEED;
}
