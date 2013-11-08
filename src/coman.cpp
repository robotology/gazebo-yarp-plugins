/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Mingo Enrico, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include <coman.h>

#include <yarp/sig/all.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/os/all.h>
#include <boost/archive/text_iarchive.hpp>


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::sig::file;
using namespace yarp::dev;



void coman::gazebo_init()
{
    //_robot = gazebo_pointer_wrapper::getModel();
    std::cout<<"if this message is the last one you read, _robot has not been set"<<std::endl;
    assert ( _robot );

    std::cout<<"Robot Name: "<<_robot->GetName() <<std::endl;
    std::cout<<"# Joints: "<<_robot->GetJoints().size() <<std::endl;
    std::cout<<"# Links: "<<_robot->GetLinks().size() <<std::endl;



    this->robot_refresh_period=this->_robot->GetWorld()->GetPhysicsEngine()->GetUpdatePeriod() *1000.0;
    setJointNames();

    _robot_number_of_joints = joint_names.size();
    pos_lock.unlock();
    pos.size ( _robot_number_of_joints );
    zero_pos.size ( _robot_number_of_joints );
    vel.size ( _robot_number_of_joints );
    speed.size ( _robot_number_of_joints );
    acc.size ( _robot_number_of_joints );
    amp.size ( _robot_number_of_joints );
    torque.size ( _robot_number_of_joints );
    ref_speed.size ( _robot_number_of_joints );
    ref_pos.size ( _robot_number_of_joints );
    ref_acc.size ( _robot_number_of_joints );
    ref_torque.size ( _robot_number_of_joints );
    max_pos.resize ( _robot_number_of_joints );
    min_pos.size ( _robot_number_of_joints );
    _p.reserve ( _robot_number_of_joints );
    _i.reserve ( _robot_number_of_joints );
    _d.reserve ( _robot_number_of_joints );

    setMinMaxPos();
    setPIDs();

    pos = 0;
    zero_pos=0;
    vel = 0;
    speed = 0;
    ref_speed=0;
    ref_pos=0;
    ref_acc=0;
    ref_torque=0;
    acc = 0;
    amp = 1; // initially on - ok for simulator
    started=false;
    control_mode=new int[_robot_number_of_joints];
    motion_done=new bool[_robot_number_of_joints];
    _clock=0;
    for ( int j=0; j<_robot_number_of_joints; ++j )
        control_mode[j]=VOCAB_CM_POSITION;

    this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin (
                                 boost::bind ( &coman::onUpdate, this, _1 ) );
    gazebo_node_ptr = gazebo::transport::NodePtr ( new gazebo::transport::Node );
    gazebo_node_ptr->Init ( this->_robot->GetWorld()->GetName() );
    jointCmdPub = gazebo_node_ptr->Advertise<gazebo::msgs::JointCmd>
                  ( std::string ( "~/" ) + this->_robot->GetName() + "/joint_cmd" );


    _T_controller = 10;


}


void coman::onUpdate ( const gazebo::common::UpdateInfo & /*_info*/ )
{
    if ( !started )
    {
        started=true;
        double temp=0;//[_robot_number_of_joints];
        for ( unsigned int j=0; j<_robot_number_of_joints; j++ )
            sendPositionToGazebo ( j,temp );
    }

    pos_lock.lock();

    // Sensing position & torque
    for ( int jnt_cnt=0; jnt_cnt < joint_names.size(); jnt_cnt++ )
    {
        /** \todo consider multi-dof joint ? */
        pos[jnt_cnt] = this->_robot->GetJoint ( joint_names[jnt_cnt] )->GetAngle ( 0 ).Degree();
        speed[jnt_cnt] = this->_robot->GetJoint ( joint_names[jnt_cnt] )->GetVelocity ( 0 );
        torque[jnt_cnt] = this->_robot->GetJoint ( joint_names[jnt_cnt] )->GetForce ( 0 );
    }

    pos_lock.unlock();

    _clock++;

    for ( unsigned int j=0; j<_robot_number_of_joints; ++j )
    {
        /*if (control_mode[j]==VOCAB_CM_POSITION)
         *       {
         *           sendPositionToGazebo(j,ref_pos[j]);
         }*/

        if ( control_mode[j]==VOCAB_CM_POSITION ) //set pos joint value, set vel joint value
        {

            if ( _clock%_T_controller==0 )
            {
                double temp=ref_pos[j];
                if ( ( pos[j]-ref_pos[j] ) < -ROBOT_POSITION_TOLERANCE )
                {
                    if ( ref_speed[j]!=0 ) temp=pos[j]+ ( ref_speed[j]/1000.0 ) *robot_refresh_period* ( double ) _T_controller;
                    motion_done[j]=false;
                }
                else if ( ( pos[j]-ref_pos[j] ) >ROBOT_POSITION_TOLERANCE )
                {
                    if ( ref_speed[j]!=0 ) temp=pos[j]- ( ref_speed[j]/1000.0 ) *robot_refresh_period* ( double ) _T_controller;
                    motion_done[j]=false;
                }
                else
                    motion_done[j]=true;

                //            std::cout<<"pos: "<<pos[j]<<" ref_pos: "<<ref_pos[j]<<" ref_speed: "<<ref_speed[j]<<" period: "<<robot_refresh_period<<" result: "<<temp<<std::endl;
                sendPositionToGazebo ( j,temp );
            }
        }
        else if ( control_mode[j]==VOCAB_CM_VELOCITY ) //set vmo joint value
        {
            if ( _clock%_T_controller==0 )
            {
                sendVelocityToGazebo ( j,vel[j] );
                //std::cout<<" velocity "<<vel[j]<<'('<<toRad(vel[j])<<')'<<" to joint "<<j<<std::endl;
            }
        }
        else if ( control_mode[j]==VOCAB_CM_TORQUE )
        {
            if ( _clock%_T_controller==0 )
            {
                sendTorqueToGazebo ( j,ref_torque[j] );
                //std::cout<<" torque "<<ref_torque[j]<<" to joint "<<j<<std::endl;
            }
        }
    }
}


