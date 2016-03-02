/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: Marco Randazzo <marco.randazzo@iit.it>
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_COUPLING_H
#define GAZEBOYARP_COUPLING_H

#include <gazebo/physics/physics.hh>

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
    unsigned int m_controllerPeriod;
    BaseCouplingHandler(gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints);
    
public:
    virtual ~BaseCouplingHandler();

    virtual bool decouplePos(yarp::sig::Vector& current_pos) = 0;
    virtual bool decoupleVel(yarp::sig::Vector& current_vel) = 0;
    virtual bool decoupleAcc(yarp::sig::Vector& current_acc) = 0;
    virtual bool decoupleTrq(yarp::sig::Vector& current_trq) = 0;

    virtual yarp::sig::VectorOf<int> getCoupledJoints();
    virtual bool checkJointIsCoupled(int joint);

    virtual yarp::sig::Vector decoupleRefPos (yarp::sig::Vector& pos_ref) = 0;
    virtual yarp::sig::Vector decoupleRefVel (yarp::sig::Vector& vel_ref) = 0;
    virtual yarp::sig::Vector decoupleRefTrq (yarp::sig::Vector& trq_ref) = 0;
};

class EyesCouplingHandler : public BaseCouplingHandler
{

public:
    EyesCouplingHandler(gazebo::physics::Model* model, yarp::sig::VectorOf<int> coupled_joints);

public:
    bool decouplePos (yarp::sig::Vector& current_pos);
    bool decoupleVel (yarp::sig::Vector& current_vel);
    bool decoupleAcc (yarp::sig::Vector& current_acc);
    bool decoupleTrq (yarp::sig::Vector& current_trq);

    yarp::sig::Vector decoupleRefPos (yarp::sig::Vector& pos_ref);
    yarp::sig::Vector decoupleRefVel (yarp::sig::Vector& vel_ref);
    yarp::sig::Vector decoupleRefTrq (yarp::sig::Vector& trq_ref);
};

#endif //GAZEBOYARP_COUPLING_H
