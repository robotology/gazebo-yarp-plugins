/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include "gazebo_yarp_plugins/ControlBoardDriver.h"



using namespace yarp::dev;

bool GazeboYarpControlBoardDriver::setPositionMode(int j) //WORKS
{
    /* WARNING: disabling velocity mode. This is needed as long as we use
     *               the SetVelocity method for velocity control
    if(controlMode[j]==VOCAB_CM_VELOCITY) {
        gazebo::physics::JointPtr joint =  this->_robot->GetJoint(joint_names[j]);
        joint->SetMaxForce(0, 0);
        joint->SetVelocity(0,0);
    }*/
    
    // resetting controller PIDs
    this->_robot->GetJointController()->AddJoint(this->_robot->GetJoint(joint_names[j]));
    controlMode[j]=VOCAB_CM_POSITION;
    std::cout<<"control mode = position "<<j<<std::endl;
    return true;
}

bool GazeboYarpControlBoardDriver::setVelocityMode(int j) //WORKS
 {
     /* TODO: this is needed if we want to control velocities using JointController
      *   // resetting controller PIDs
      */   this->_robot->GetJointController()->AddJoint(this->_robot->GetJoint(joint_names[j]));
      /**/
     controlMode[j]=VOCAB_CM_VELOCITY;
     std::cout<<"control mode = speed "<<j<<std::endl;
     return true;
 }
 
 bool GazeboYarpControlBoardDriver::getControlMode(int j, int *mode) //WORKS
 {
     *mode=controlMode[j];
     return true;
 }
 
 bool GazeboYarpControlBoardDriver::getControlModes(int *modes) //NOT TESTED
 {
     for(unsigned int j=0; j<numberOfJoints; ++j)
     {
         modes[j]=controlMode[j];
     }
     return true;
 }
 
 bool GazeboYarpControlBoardDriver::setTorqueMode(int j) //NOT TESTED
 {
     /* WARNING: disabling velocity mode. This is needed as long as we use
      *               the SetVelocity method for velocity control*/
     if(controlMode[j]==VOCAB_CM_VELOCITY) {
         gazebo::physics::JointPtr joint =  this->_robot->GetJoint(joint_names[j]);
         joint->SetMaxForce(0, 0);
         joint->SetVelocity(0,0);
     }
     
     controlMode[j]=VOCAB_CM_TORQUE;
     std::cout<<"control mode = torque "<<j<<std::endl;
     return true;
 }
 

 bool GazeboYarpControlBoardDriver::setImpedancePositionMode(int j)//NOT TESTED
 {
     /* WARNING: disabling velocity mode. This is needed as long as we use
      *               the SetVelocity method for velocity control*/
     if(controlMode[j]==VOCAB_CM_VELOCITY) {
         gazebo::physics::JointPtr joint =  this->_robot->GetJoint(joint_names[j]);
         joint->SetMaxForce(0, 0);
         joint->SetVelocity(0,0);
     }

     controlMode[j]=VOCAB_CM_IMPEDANCE_POS;
     std::cout<<"control mode = impedance position "<<j<<std::endl;
     return true;
 }
 bool GazeboYarpControlBoardDriver::setImpedanceVelocityMode(int) //NOT IMPLEMENTED
 {
     return false;
 }
 bool GazeboYarpControlBoardDriver::setOpenLoopMode(int j) //NOT IMPLEMENTED
 {
     if(controlMode[j] == VOCAB_CM_VELOCITY) {
         gazebo::physics::JointPtr joint =  this->_robot->GetJoint(joint_names[j]);
         joint->SetMaxForce(0, 0);
         joint->SetVelocity(0,0);
     }
     
     controlMode[j] = VOCAB_CM_OPENLOOP;
     std::cout<<"control mode = openloop "<<j<<std::endl;
     return true;
 }

