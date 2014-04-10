/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include "gazebo_yarp_plugins/ControlBoardDriver.h"



using namespace yarp::dev;

void GazeboYarpControlBoardDriver::prepareResetJointMsg(int j)
{
    gazebo::msgs::JointCmd j_cmd;
    j_cmd.set_reset(true);
    j_cmd.set_name(this->_robot->GetJoint(joint_names[j])->GetScopedName());
    this->jointCmdPub->WaitForConnection();
    this->jointCmdPub->Publish(j_cmd);
}

bool GazeboYarpControlBoardDriver::setPositionMode(int j) //WORKS
{
    prepareResetJointMsg(j);
    controlMode[j]=VOCAB_CM_POSITION;
    std::cout<<"control mode = position "<<j<<std::endl;
    return true;
}

bool GazeboYarpControlBoardDriver::setVelocityMode(int j) //WORKS
 {
    prepareResetJointMsg(j);
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
    prepareResetJointMsg(j);
    controlMode[j]=VOCAB_CM_TORQUE;
    std::cout<<"control mode = torque "<<j<<std::endl;
    return true;
 }


 bool GazeboYarpControlBoardDriver::setImpedancePositionMode(int j)//NOT TESTED
 {
    prepareResetJointMsg(j);
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
     prepareResetJointMsg(j);
     controlMode[j] = VOCAB_CM_OPENLOOP;
     std::cout<<"control mode = openloop "<<j<<std::endl;
     return true;
 }

