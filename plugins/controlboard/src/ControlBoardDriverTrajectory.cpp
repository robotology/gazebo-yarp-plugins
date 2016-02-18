/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: Marco Randazzo <marco.randazzo@iit.it>
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "ControlBoardDriver.h"
#include <GazeboYarpPlugins/common.h>

#include <ControlBoardDriverTrajectory.h>
#include <cstdio>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/math/Angle.hh>

#include <yarp/os/LogStream.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

//------------------------------------------------------------------------------------------------------------------
// MinJerkTrajectoryGenerator
//------------------------------------------------------------------------------------------------------------------

double MinJerkTrajectoryGenerator::compute_p5f (double t)
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

double MinJerkTrajectoryGenerator::compute_p5f_vel (double t)
{
  double accum = -2*m_dx0*t - m_dx0;
  accum = accum + (30*m_x0 +15*m_dx0    -30*m_xf) * (t*t);
  accum = -accum/m_tf * (t-1)*(t-1);
  return accum;
}

double MinJerkTrajectoryGenerator::compute_current_vel()
{
  double a;

  //(10 * (t/T)^3 - 15 * (t/T)^4 + 6 * (t/T)^5) * (x0-xf) + x0
  if (m_trajectory_complete) return 0;
  if (m_cur_t == 0)
  {
    return 0;
  }
  else
  if (m_cur_t < 1.0 - m_step)
  {
    // calculate the velocity 
    a = compute_p5f_vel (m_cur_t);
    return a;
  }
  return 0;
}

bool MinJerkTrajectoryGenerator::abortTrajectory (double limit)
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

bool  MinJerkTrajectoryGenerator::initTrajectory (double current_pos, double final_pos, double speed)
{
  m_controllerPeriod = (unsigned)(this->m_robot->GetWorld()->GetPhysicsEngine()->GetUpdatePeriod() * 1000.0); 
  double speedf = fabs(speed);
  double dx0 =0;
  m_computed_reference = current_pos;

  if (speed <= 0) return false;

  m_dx0 = compute_current_vel();
  m_x0 = current_pos;
  m_prev_a = current_pos;
  m_xf = final_pos;
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
    abortTrajectory (final_pos);
    m_step=0;
    return false;
  }
  else
  {
    m_step = 1 / m_tf;
  }
  m_cur_t = 0;
  m_cur_step = 0;
  m_trajectory_complete =  false;
  
  return true;
}

double MinJerkTrajectoryGenerator::computeTrajectory()
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
    target = compute_p5f (m_cur_t);
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

//------------------------------------------------------------------------------------------------------------------
// ConstSpeedTrajectoryGenerator
//------------------------------------------------------------------------------------------------------------------

bool ConstSpeedTrajectoryGenerator::initTrajectory (double current_pos, double final_pos, double speed)
{
  m_controllerPeriod = (unsigned)(this->m_robot->GetWorld()->GetPhysicsEngine()->GetUpdatePeriod() * 1000.0); 
  m_x0 = current_pos;
  m_xf = final_pos;
  m_speed = speed;
  m_computed_reference = m_x0;
  return true;
}

bool ConstSpeedTrajectoryGenerator::abortTrajectory(double limit)
{
  return true;
}

double ConstSpeedTrajectoryGenerator::computeTrajectory()
{
    double step = (m_speed / 1000.0) * m_controllerPeriod; //* _T_controller;
    double error_abs = fabs(m_computed_reference - m_xf);

    // if delta is bigger then threshold, in some cases this will never converge to an end.
    // Check to prevent those cases
    if(error_abs)
    {
        // Watch out for problem
        //if((error_abs < m_positionThreshold) || ( error_abs < step) )    // This id both 'normal ending condition' and safe procedure when step > threshold causing infinite oscillation around final position
        if( error_abs < step)  
        {
           // Just go to final position
           m_computed_reference = m_xf;
           m_trajectory_complete = true;
        }
        else
        {
          if (m_xf > m_computed_reference)
          {
             m_computed_reference += step;
             m_trajectory_complete = false;
          }
          else
          {
             m_computed_reference -= step;
             m_trajectory_complete = false;
          }
        }
    }
    return m_computed_reference;
}