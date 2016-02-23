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

  unsigned int m_controllerPeriod;
  BaseCouplingHandler(gazebo::physics::Model* model) {m_robot = model;}
  
public:
  virtual bool decouplePos (yarp::sig::Vector& current_pos)=0;
  virtual bool decoupleVel (yarp::sig::Vector& current_vel)=0;
  virtual bool decoupleAcc (yarp::sig::Vector& current_acc)=0;
  virtual bool decoupleTrq (yarp::sig::Vector& current_trq)=0;  
};

class EyesCouplingHandler : public BaseCouplingHandler
{
 
public:
  EyesCouplingHandler(gazebo::physics::Model* model):BaseCouplingHandler(model) {}
  
public:
  bool decouplePos (yarp::sig::Vector& current_pos);
  bool decoupleVel (yarp::sig::Vector& current_vel);
  bool decoupleAcc (yarp::sig::Vector& current_acc);
  bool decoupleTrq (yarp::sig::Vector& current_trq);  
};

#endif //GAZEBOYARP_COUPLING_H
