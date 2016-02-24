/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: Marco Randazzo <marco.randazzo@iit.it>
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_TRAJECTORY_H
#define GAZEBOYARP_TRAJECTORY_H

#include <gazebo/physics/physics.hh>

namespace yarp {
  namespace dev {
    enum TrajectoryType
    {
      TRAJECTORY_TYPE_CONST_SPEED = 0,
      TRAJECTORY_TYPE_MIN_JERK = 1
    };
  }
}
  
class TrajectoryGenerator
{
protected:
  gazebo::physics::Model* m_robot;
  bool   m_trajectory_complete;
  double m_x0;
  double m_xf;
  double m_speed;
  double m_computed_reference;
  unsigned int m_controllerPeriod;
  TrajectoryGenerator(gazebo::physics::Model* model) {m_robot = model; m_trajectory_complete=true; m_speed=0;}
  
public:
  virtual bool initTrajectory (double current_pos, double final_pos, double speed)=0;
  virtual bool abortTrajectory(double limit)=0;
  virtual double computeTrajectory()=0;  
  virtual double computeTrajectoryStep()=0;
  virtual yarp::dev::TrajectoryType getTrajectoryType()=0;
  bool isMotionDone() {return m_trajectory_complete;}
};

class ConstSpeedTrajectoryGenerator: public TrajectoryGenerator
{
public:
  ConstSpeedTrajectoryGenerator(gazebo::physics::Model* model):TrajectoryGenerator(model) {}
  ~ConstSpeedTrajectoryGenerator();  
  
public:
  bool initTrajectory (double current_pos, double final_pos, double speed);
  bool abortTrajectory(double limit);
  double computeTrajectory();
  double computeTrajectoryStep();
  yarp::dev::TrajectoryType getTrajectoryType() {return yarp::dev::TRAJECTORY_TYPE_CONST_SPEED;}
};

class MinJerkTrajectoryGenerator: public TrajectoryGenerator
{
  
public:
  MinJerkTrajectoryGenerator(gazebo::physics::Model* model):TrajectoryGenerator(model) {}
  ~MinJerkTrajectoryGenerator();
  
private:
  double m_trajectory_coeff_c1;
  double m_trajectory_coeff_c2;
  double m_trajectory_coeff_c3;
  double m_dx0;
  double m_tf;
  double m_prev_a;
  double m_cur_t;
  double m_cur_step;
  double m_step;

  double compute_p5f (double t);
  double compute_p5f_vel (double t);
  double compute_current_vel();
  
public:
  bool initTrajectory (double current_pos, double final_pos, double speed);
  bool abortTrajectory(double limit);
  double computeTrajectory();
  double computeTrajectoryStep();
  yarp::dev::TrajectoryType getTrajectoryType() {return yarp::dev::TRAJECTORY_TYPE_MIN_JERK;}
};

#endif //GAZEBOYARP_TRAJECTORY_H
