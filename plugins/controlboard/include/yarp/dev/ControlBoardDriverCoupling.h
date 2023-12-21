/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: Marco Randazzo <marco.randazzo@iit.it>
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_COUPLING_H
#define GAZEBOYARP_COUPLING_H

#include <ControlBoardDriverRange.h>

#include <gazebo/physics/Model.hh>

#include <unordered_map>

namespace yarp {
    namespace dev {
        enum CouplingType
        {
            COUPLING_NONE = 0
        };
    }
}

class BaseCouplingHandler
{
protected:
    gazebo::physics::Model* m_robot;
    yarp::sig::VectorOf<int> m_coupledJoints;
    std::vector<std::string> m_coupledJointNames;
    std::unordered_map<int, Range> m_coupledJointLimits;
    unsigned int m_controllerPeriod;
    unsigned int m_couplingSize;
    BaseCouplingHandler(gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names, std::vector<Range> coupled_joint_limits);

public:
    virtual ~BaseCouplingHandler();

    virtual bool decouplePos(yarp::sig::Vector& current_pos) = 0;
    virtual bool decoupleVel(yarp::sig::Vector& current_vel) = 0;
    virtual bool decoupleAcc(yarp::sig::Vector& current_acc) = 0;
    virtual bool decoupleTrq(yarp::sig::Vector& current_trq) = 0;

    virtual yarp::sig::VectorOf<int> getCoupledJoints();
    virtual std::string getCoupledJointName (int joint);
    virtual bool checkJointIsCoupled(int joint);

    virtual yarp::sig::Vector decoupleRefPos (yarp::sig::Vector& pos_ref) = 0;
    virtual yarp::sig::Vector decoupleRefVel (yarp::sig::Vector& vel_ref, const yarp::sig::Vector& pos_feedback) = 0;
    virtual yarp::sig::Vector decoupleRefTrq (yarp::sig::Vector& trq_ref) = 0;

    virtual void setCoupledJointLimit(int joint, const double& min, const double& max);
    virtual void getCoupledJointLimit(int joint, double& min, double& max);
};

class EyesCouplingHandler : public BaseCouplingHandler
{

public:
    EyesCouplingHandler(gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names, std::vector<Range> coupled_joint_limits);

public:
    bool decouplePos (yarp::sig::Vector& current_pos);
    bool decoupleVel (yarp::sig::Vector& current_vel);
    bool decoupleAcc (yarp::sig::Vector& current_acc);
    bool decoupleTrq (yarp::sig::Vector& current_trq);

    yarp::sig::Vector decoupleRefPos (yarp::sig::Vector& pos_ref);
    yarp::sig::Vector decoupleRefVel (yarp::sig::Vector& vel_ref, const yarp::sig::Vector& pos_feedback);
    yarp::sig::Vector decoupleRefTrq (yarp::sig::Vector& trq_ref);
};

class ThumbCouplingHandler : public BaseCouplingHandler
{

public:
    ThumbCouplingHandler (gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names, std::vector<Range> coupled_joint_limits);

public:
    bool decouplePos (yarp::sig::Vector& current_pos);
    bool decoupleVel (yarp::sig::Vector& current_vel);
    bool decoupleAcc (yarp::sig::Vector& current_acc);
    bool decoupleTrq (yarp::sig::Vector& current_trq);

    yarp::sig::Vector decoupleRefPos (yarp::sig::Vector& pos_ref);
    yarp::sig::Vector decoupleRefVel (yarp::sig::Vector& vel_ref, const yarp::sig::Vector& pos_feedback);
    yarp::sig::Vector decoupleRefTrq (yarp::sig::Vector& trq_ref);
};

class IndexCouplingHandler : public BaseCouplingHandler
{

public:
    IndexCouplingHandler (gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names, std::vector<Range> coupled_joint_limits);

public:
    bool decouplePos (yarp::sig::Vector& current_pos);
    bool decoupleVel (yarp::sig::Vector& current_vel);
    bool decoupleAcc (yarp::sig::Vector& current_acc);
    bool decoupleTrq (yarp::sig::Vector& current_trq);

    yarp::sig::Vector decoupleRefPos (yarp::sig::Vector& pos_ref);
    yarp::sig::Vector decoupleRefVel (yarp::sig::Vector& vel_ref, const yarp::sig::Vector& pos_feedback);
    yarp::sig::Vector decoupleRefTrq (yarp::sig::Vector& trq_ref);
};

class MiddleCouplingHandler : public BaseCouplingHandler
{

public:
    MiddleCouplingHandler (gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names, std::vector<Range> coupled_joint_limits);

public:
    bool decouplePos (yarp::sig::Vector& current_pos);
    bool decoupleVel (yarp::sig::Vector& current_vel);
    bool decoupleAcc (yarp::sig::Vector& current_acc);
    bool decoupleTrq (yarp::sig::Vector& current_trq);

    yarp::sig::Vector decoupleRefPos (yarp::sig::Vector& pos_ref);
    yarp::sig::Vector decoupleRefVel (yarp::sig::Vector& vel_ref, const yarp::sig::Vector& pos_feedback);
    yarp::sig::Vector decoupleRefTrq (yarp::sig::Vector& trq_ref);
};

class PinkyCouplingHandler : public BaseCouplingHandler
{

public:
    PinkyCouplingHandler (gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names, std::vector<Range> coupled_joint_limits);

public:
    bool decouplePos (yarp::sig::Vector& current_pos);
    bool decoupleVel (yarp::sig::Vector& current_vel);
    bool decoupleAcc (yarp::sig::Vector& current_acc);
    bool decoupleTrq (yarp::sig::Vector& current_trq);

    yarp::sig::Vector decoupleRefPos (yarp::sig::Vector& pos_ref);
    yarp::sig::Vector decoupleRefVel (yarp::sig::Vector& vel_ref, const yarp::sig::Vector& pos_feedback);
    yarp::sig::Vector decoupleRefTrq (yarp::sig::Vector& trq_ref);
};

class FingersAbductionCouplingHandler : public BaseCouplingHandler
{

public:
    FingersAbductionCouplingHandler (gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names, std::vector<Range> coupled_joint_limits);

public:
    bool decouplePos (yarp::sig::Vector& current_pos);
    bool decoupleVel (yarp::sig::Vector& current_vel);
    bool decoupleAcc (yarp::sig::Vector& current_acc);
    bool decoupleTrq (yarp::sig::Vector& current_trq);

    yarp::sig::Vector decoupleRefPos (yarp::sig::Vector& pos_ref);
    yarp::sig::Vector decoupleRefVel (yarp::sig::Vector& vel_ref, const yarp::sig::Vector& pos_feedback);
    yarp::sig::Vector decoupleRefTrq (yarp::sig::Vector& trq_ref);
};

class CerHandCouplingHandler : public BaseCouplingHandler
{

public:
    CerHandCouplingHandler (gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names, std::vector<Range> coupled_joint_limits);

public:
    bool decouplePos (yarp::sig::Vector& current_pos);
    bool decoupleVel (yarp::sig::Vector& current_vel);
    bool decoupleAcc (yarp::sig::Vector& current_acc);
    bool decoupleTrq (yarp::sig::Vector& current_trq);

    yarp::sig::Vector decoupleRefPos (yarp::sig::Vector& pos_ref);
    yarp::sig::Vector decoupleRefVel (yarp::sig::Vector& vel_ref, const yarp::sig::Vector& pos_feedback);
    yarp::sig::Vector decoupleRefTrq (yarp::sig::Vector& trq_ref);
};

class HandMk3CouplingHandler : public BaseCouplingHandler
{

public:
    HandMk3CouplingHandler (gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names, std::vector<Range> coupled_joint_limits);

public:
    bool decouplePos (yarp::sig::Vector& current_pos);
    bool decoupleVel (yarp::sig::Vector& current_vel);
    bool decoupleAcc (yarp::sig::Vector& current_acc);
    bool decoupleTrq (yarp::sig::Vector& current_trq);

    yarp::sig::Vector decoupleRefPos (yarp::sig::Vector& pos_ref);
    yarp::sig::Vector decoupleRefVel (yarp::sig::Vector& vel_ref, const yarp::sig::Vector& pos_feedback);
    yarp::sig::Vector decoupleRefTrq (yarp::sig::Vector& trq_ref);

protected:
    double decouple (double q2, std::vector<double>& lut);

    const int LUTSIZE;

    std::vector<double> thumb_lut;
    std::vector<double> index_lut;
};

class HandMk4CouplingHandler : public BaseCouplingHandler
{

public:
    HandMk4CouplingHandler (gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names, std::vector<Range> coupled_joint_limits);

public:
    bool decouplePos (yarp::sig::Vector& current_pos);
    bool decoupleVel (yarp::sig::Vector& current_vel);
    bool decoupleAcc (yarp::sig::Vector& current_acc);
    bool decoupleTrq (yarp::sig::Vector& current_trq);

    yarp::sig::Vector decoupleRefPos (yarp::sig::Vector& pos_ref);
    yarp::sig::Vector decoupleRefVel (yarp::sig::Vector& vel_ref, const yarp::sig::Vector& pos_feedback);
    yarp::sig::Vector decoupleRefTrq (yarp::sig::Vector& trq_ref);

protected:
    double decouple (double q2, std::vector<double>& lut);

    const int LUTSIZE;

    std::vector<double> thumb_lut;
    std::vector<double> index_lut;
    std::vector<double> pinkie_lut;
};

#endif //GAZEBOYARP_COUPLING_H
