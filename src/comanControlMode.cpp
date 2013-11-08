#include "coman.h"


using namespace yarp::dev;

bool coman::setPositionMode(int j) //WORKS
{
    /* WARNING: disabling velocity mode. This is needed as long as we use
     *               the SetVelocity method for velocity control*/
    if(control_mode[j]==VOCAB_CM_VELOCITY) {
        gazebo::physics::JointPtr joint =  this->_robot->GetJoint(joint_names[j]);
        joint->SetMaxForce(0, 0);
        joint->SetVelocity(0,0);
    }
    
    // resetting controller PIDs
    this->_robot->GetJointController()->AddJoint(this->_robot->GetJoint(joint_names[j]));
    control_mode[j]=VOCAB_CM_POSITION;
    std::cout<<"control mode = position "<<j<<std::endl;
}

bool coman::setVelocityMode(int j) //WORKS
 {
     /* TODO: this is needed if we want to control velocities using JointController
      *   // resetting controller PIDs
      *   this->_robot->GetJointController()->AddJoint(this->_robot->GetJoint(joint_names[j]));
      */
     control_mode[j]=VOCAB_CM_VELOCITY;
     std::cout<<"control mode = speed "<<j<<std::endl;
 }
 
 bool coman::getControlMode(int j, int *mode) //WORKS
 {
     mode[j]=control_mode[j];
 }
 
 bool coman::getControlModes(int *modes) //NOT TESTED
 {
     for(int j=0; j<_robot_number_of_joints; ++j)
     {
         modes[j]=control_mode[j];
     }
     return true;
 }
 
 bool coman::setTorqueMode(int j) //NOT TESTED
 {
     /* WARNING: disabling velocity mode. This is needed as long as we use
      *               the SetVelocity method for velocity control*/
     if(control_mode[j]==VOCAB_CM_VELOCITY) {
         gazebo::physics::JointPtr joint =  this->_robot->GetJoint(joint_names[j]);
         joint->SetMaxForce(0, 0);
         joint->SetVelocity(0,0);
     }
     
     control_mode[j]=VOCAB_CM_TORQUE;
     std::cout<<"control mode = torque "<<j<<std::endl;
 }
 

 bool coman::setImpedancePositionMode(int j)//NOT IMPLEMENTED
 {
     return false;
 }
 bool coman::setImpedanceVelocityMode(int j) //NOT IMPLEMENTED
 {
     return false;
 }
 bool coman::setOpenLoopMode(int j) //NOT IMPLEMENTED
 {
     return false;
 }

