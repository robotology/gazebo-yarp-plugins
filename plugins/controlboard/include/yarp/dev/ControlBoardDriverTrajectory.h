/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: Marco Randazzo <marco.randazzo@iit.it>
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_TRAJECTORY_H
#define GAZEBOYARP_TRAJECTORY_H

#include <gazebo/physics/Model.hh>

namespace yarp {
    namespace dev {
        enum TrajectoryType
        {
            TRAJECTORY_TYPE_CONST_SPEED = 0,
            TRAJECTORY_TYPE_MIN_JERK = 1
        };
    }
}

class Watchdog
{
    double m_duration;
    double m_lastUpdate;
public:
    void   reset();
    bool   isExpired();
    void   modifyDuration(double expireTime);
    double getDuration();

    Watchdog (double expireTime);
};

class RampFilter
{
private:
    yarp::os::Semaphore m_mutex;
    double m_final_reference;
    double m_current_value;
    double m_step;
public:

    RampFilter();

    void setReference(double ref, double step);
    void update();
    double getCurrentValue();
    void stop();
};

class TrajectoryGenerator
{
protected:
    yarp::os::Semaphore m_mutex;
    gazebo::physics::Model* m_robot;
    bool   m_trajectory_complete;
    double m_x0;
    double m_xf;
    double m_speed;
    double m_computed_reference;
    double m_controllerPeriod;
    double m_joint_min;
    double m_joint_max;
    TrajectoryGenerator(gazebo::physics::Model* model);

public:
    virtual ~TrajectoryGenerator();
    virtual bool initTrajectory (double current_pos, double final_pos, double speed) = 0;
    virtual bool abortTrajectory(double limit) = 0;
    virtual double computeTrajectory() = 0;
    virtual double computeTrajectoryStep() = 0;
    virtual yarp::dev::TrajectoryType getTrajectoryType() = 0;
    bool setLimits(double min, double max);
    bool isMotionDone();
};

class ConstSpeedTrajectoryGenerator: public TrajectoryGenerator
{
public:
    ConstSpeedTrajectoryGenerator(gazebo::physics::Model* model);
    virtual ~ConstSpeedTrajectoryGenerator();

private:
    double p_computeTrajectory();
    double p_computeTrajectoryStep();
    bool   p_abortTrajectory (double limit);
      
public:
    bool initTrajectory(double current_pos, double final_pos, double speed);
    bool abortTrajectory(double limit);
    double computeTrajectory();
    double computeTrajectoryStep();
    yarp::dev::TrajectoryType getTrajectoryType();
};

class MinJerkTrajectoryGenerator: public TrajectoryGenerator
{
public:
    MinJerkTrajectoryGenerator(gazebo::physics::Model* model);
    virtual ~MinJerkTrajectoryGenerator();

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

    double p_computeTrajectory();
    double p_computeTrajectoryStep();
    bool   p_abortTrajectory (double limit);
        
    double p_compute_p5f(double t);
    double p_compute_p5f_vel(double t);
    double p_compute_current_vel();

public:
    bool initTrajectory(double current_pos, double final_pos, double speed);
    bool abortTrajectory(double limit);
    double computeTrajectory();
    double computeTrajectoryStep();
    yarp::dev::TrajectoryType getTrajectoryType();
};

#endif //GAZEBOYARP_TRAJECTORY_H
