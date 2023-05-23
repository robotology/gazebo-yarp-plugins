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

class HandMk5CouplingHandler : public BaseCouplingHandler
{

public:
    HandMk5CouplingHandler (gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints, std::vector<std::string> coupled_joint_names, std::vector<Range> coupled_joint_limits);

public:
    bool decouplePos (yarp::sig::Vector& current_pos);
    bool decoupleVel (yarp::sig::Vector& current_vel);
    bool decoupleAcc (yarp::sig::Vector& current_acc);
    bool decoupleTrq (yarp::sig::Vector& current_trq);

    yarp::sig::Vector decoupleRefPos (yarp::sig::Vector& pos_ref);
    yarp::sig::Vector decoupleRefVel (yarp::sig::Vector& vel_ref, const yarp::sig::Vector& pos_feedback);
    yarp::sig::Vector decoupleRefTrq (yarp::sig::Vector& trq_ref);

private:
    /**
     * Parameters from https://icub-tech-iit.github.io/documentation/hands/hands_mk5_coupling
     */
    struct FingerParameters
    {
        double L0x;
        double L0y;
        double L1x;
        double L1y;
        double q2bias;
        double q1off;
        double k;
        double d;
        double l;
        double b;
    };

    const FingerParameters mParamsThumb =
    {
        .L0x = -0.00555,
        .L0y = 0.00285,
        .q2bias = 180.0,
        .q1off = 4.29,
        .k = 0.0171,
        .d = 0.02006,
        .l = 0.0085,
        .b = 0.00624
    };

    const FingerParameters mParamsIndex =
    {
        .L0x = -0.0050,
        .L0y = 0.0040,
        .q2bias = 173.35,
        .q1off = 2.86,
        .k = 0.02918,
        .d = 0.03004,
        .l = 0.00604,
        .b = 0.0064
    };

    const FingerParameters mParamsPinky =
    {
        .L0x = -0.0050,
        .L0y = 0.0040,
        .q2bias = 170.54,
        .q1off = 3.43,
        .k = 0.02425,
        .d = 0.02504,
        .l = 0.00608,
        .b = 0.0064
    };

    /*
     * This method implements the law q2 = q2(q1) from
     * https://icub-tech-iit.github.io/documentation/hands/hands_mk5_coupling,
     * i.e., the absolute angle of the distal joint q2 with respect to the palm.
     *
     * The inputs q1 and the return value of the function are in degrees.
     */
    double evaluateCoupledJoint(const double& q1, const FingerParameters& parameters);

    /*
     * This method evalutes the relative angle between the proximal and distal joints of the thumb finger.
     *
     * The input q1 and the return value of the function are in degrees.
     */
    double evaluateCoupledJointThumb(const double& q1);

    /*
     * This method evalutes the relative angle between the proximal and distal joints of the index finger.
     * This is also valid for the middle and ring fingers.
     *
     * The input q1 and the return value of the function are in degrees.
     */
    double evaluateCoupledJointIndex(const double& q1);

    /*
     * This method evalutes the relative angle between the proximal and distal joints of the pinky finger.
     *
     * The input q1 and the return value of the function are in degrees.
     */
    double evaluateCoupledJointPinky(const double& q1);

    /*
     * This method implements the law \frac{\partial{q2}}{\partial{q1}} from
     * https://icub-tech-iit.github.io/documentation/hands/hands_mk5_coupling,
     * i.e., the jacobian of the absolute angle of the distal joint q2 measured from the palm,
     * with respect to the proximal joint q1.
     *
     * The input q1 is in degrees.
     */
    double evaluateCoupledJointJacobian(const double& q1, const FingerParameters& parameters);

    /*
     * This method evalutes the jacobian of the relative angle between the proximal and distal joints of the thumb finger
     * with respect to the proximal joint.
     *
     * The input q1 is in degrees.
     */
    double evaluateCoupledJointJacobianThumb(const double& q1);

    /*
     * This method evalutes the jacobian of the relative angle between the proximal and distal joints of the index finger
     * with respect to the proximal joint.
     *
     * This is also valid for the middle and ring fingers.
     *
     * The input q1 is in degrees.
     */
    double evaluateCoupledJointJacobianIndex(const double& q1);

    /*
     * This method evalutes the jacobian of the relative angle between the proximal and distal joints of the pinky finger
     * with respect to the proximal joint.
     *
     * The input q1 is in degrees.
     */
    double evaluateCoupledJointJacobianPinky(const double& q1);
};

#endif //GAZEBOYARP_COUPLING_H
