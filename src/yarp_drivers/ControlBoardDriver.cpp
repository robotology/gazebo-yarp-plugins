/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include <gazebo_yarp_plugins/ControlBoardDriver.h>
#include "../test/jointlogger.hpp"

#include <yarp/sig/all.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/os/all.h>
#include <boost/archive/text_iarchive.hpp>
#include <stdio.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::sig::file;
using namespace yarp::dev;



void GazeboYarpControlBoardDriver::gazebo_init()
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
    //pos_lock.unlock();
    pos.size ( _robot_number_of_joints );
    zero_pos.size ( _robot_number_of_joints );
    vel.size ( _robot_number_of_joints );
    speed.size ( _robot_number_of_joints );
    acc.size ( _robot_number_of_joints );
    amp.size ( _robot_number_of_joints );
    torque.size ( _robot_number_of_joints ); torque.zero();
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
    for ( unsigned int j=0; j<_robot_number_of_joints; ++j )
        control_mode[j]=VOCAB_CM_POSITION;

    std::cout << "gazebo_init set pid done!" << std::endl;

    this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin (
                                 boost::bind ( &GazeboYarpControlBoardDriver::onUpdate, this, _1 ) );
    gazebo_node_ptr = gazebo::transport::NodePtr ( new gazebo::transport::Node );
    gazebo_node_ptr->Init ( this->_robot->GetWorld()->GetName() );
    jointCmdPub = gazebo_node_ptr->Advertise<gazebo::msgs::JointCmd>
                  ( std::string ( "~/" ) + this->_robot->GetName() + "/joint_cmd" );

    _T_controller = 10;
}


void GazeboYarpControlBoardDriver::onUpdate ( const gazebo::common::UpdateInfo & /*_info*/ )
{
    _clock++;

    if ( !started ) //This is a simple way to start with a coman in standing position
    {
        started=true;
        double temp=0;//[_robot_number_of_joints];
        for ( unsigned int j=0; j<_robot_number_of_joints; j++ )
            sendPositionToGazebo ( j,temp );
    }

    pos_lock.wait();
    // Sensing position & torque
    for ( unsigned int jnt_cnt=0; jnt_cnt < joint_names.size(); jnt_cnt++ )
    {
        /** \todo consider multi-dof joint ? */
        pos[jnt_cnt] = this->_robot->GetJoint ( joint_names[jnt_cnt] )->GetAngle ( 0 ).Degree();
        speed[jnt_cnt] = this->_robot->GetJoint ( joint_names[jnt_cnt] )->GetVelocity ( 0 );
        torque[jnt_cnt] = this->_robot->GetJoint ( joint_names[jnt_cnt] )->GetForce ( 0 );
    }
    pos_lock.post();
    
//     logger.log(speed[2]);
    
    for ( unsigned int j=0; j<_robot_number_of_joints; ++j )
    {
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
                {
                    motion_done[j]=true;
                }
                //std::cout<<"pos: "<<pos[j]<<" ref_pos: "<<ref_pos[j]<<" ref_speed: "<<ref_speed[j]<<" period: "<<robot_refresh_period<<" result: "<<temp<<std::endl;
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

void GazeboYarpControlBoardDriver::setMinMaxPos()  //NOT TESTED
{
    std::cout<<"Joint Limits"<<std::endl;
    for(unsigned int i = 0; i < _robot_number_of_joints; ++i)
    {
        max_pos[i] = this->_robot->GetJoint(joint_names[i])->GetUpperLimit(0).Degree();
        min_pos[i] = this->_robot->GetJoint(joint_names[i])->GetLowerLimit(0).Degree();
        std::cout<<joint_names[i]<<" max_pos: "<<max_pos[i]<<" min_pos: "<<min_pos[i]<<std::endl;
    }
}

void GazeboYarpControlBoardDriver::setJointNames()  //WORKS
{
        std::cout << ".ini file found, using joint names in ini file" << std::endl;
        yarp::os::Bottle joint_names_bottle = plugin_parameters.findGroup("jointNames");

        if(joint_names_bottle.isNull())
        {
            std::cout << "Error cannot find jointNames!!";
            return;
        }
        int nr_of_joints = joint_names_bottle.size()-1;
        
        joint_names.resize(nr_of_joints);
        for(unsigned int i=0; i < joint_names.size(); i++ ) {
            std::string joint_name(joint_names_bottle.get(i+1).asString().c_str());
            joint_names[i] = _robot->GetName()+"::"+joint_name;
        }        
}

void GazeboYarpControlBoardDriver::setPIDs() //WORKS
{        
    yarp::os::Property prop;
    //now try to load the pid from the plugin configuration file, if that fails fallback to the old methods
    std::string gazebo_pids_group_name = "GAZEBO_PIDS";
    
    if(plugin_parameters.check(gazebo_pids_group_name.c_str())) 
    {
        std::cout<<"Found PID information in plugin parameters "<<std::endl;
        
        for(unsigned int i = 0; i < _robot_number_of_joints; ++i)
        {
            std::stringstream property_name;
            property_name<<"Pid";
            property_name<<i;
            
            yarp::os::Bottle& pid = plugin_parameters.findGroup(gazebo_pids_group_name.c_str()).findGroup(property_name.str().c_str());
            _p.push_back(pid.get(1).asDouble());
            _i.push_back(pid.get(3).asDouble());
            _d.push_back(pid.get(2).asDouble());
            std::cout<<"  P: "<<_p[i]<<" I: "<<_i[i]<<" D: "<<_d[i]<<std::endl;
        }
        std::cout<<"OK!"<<std::endl;
    } 
    else if(prop.fromConfigFile(pid_config_abs_path.c_str()))
    {
        std::cout<<"pid.ini FOUND!"<<std::endl;
        std::string group_name = "PIDS";
        
        for(unsigned int i = 0; i < _robot_number_of_joints; ++i)
        {
            std::stringstream property_name;
            property_name<<"Pid";
            property_name<<i;
            
            yarp::os::Bottle& pid = prop.findGroup(group_name.c_str()).findGroup(property_name.str().c_str());
            _p.push_back(pid.get(1).asDouble());
            _i.push_back(pid.get(3).asDouble());
            _d.push_back(pid.get(2).asDouble());
            std::cout<<"  P: "<<_p[i]<<" I: "<<_i[i]<<" D: "<<_d[i]<<std::endl;
        }
        std::cout<<"OK!"<<std::endl;
    }
    else
    {
        std::cout<<"CAN NOT FIND pid.ini!"<<std::endl;
        for(unsigned int i = 0; i < _robot_number_of_joints; ++i)
        {
            _p.push_back(500.0);
            _i.push_back(0.1);
            _d.push_back(1.0);
        }
    }
}

bool GazeboYarpControlBoardDriver::sendPositionsToGazebo(yarp::sig::Vector refs)
{
    for (unsigned int j=0; j<_robot_number_of_joints; j++)
    {
        sendPositionToGazebo(j,refs[j]);
    }
    return true;
}

bool GazeboYarpControlBoardDriver::sendPositionToGazebo(int j,double ref)
{
    gazebo::msgs::JointCmd j_cmd;
    prepareJointMsg(j_cmd,j,ref);
    jointCmdPub->WaitForConnection();
    jointCmdPub->Publish(j_cmd);
    return true;
}

void GazeboYarpControlBoardDriver::prepareJointMsg(gazebo::msgs::JointCmd& j_cmd, const int joint_index, const double ref)  //WORKS
{
    j_cmd.set_name(this->_robot->GetJoint(joint_names[joint_index])->GetScopedName());
    j_cmd.mutable_position()->set_target(toRad(ref));
    j_cmd.mutable_position()->set_p_gain(_p[joint_index]);
    j_cmd.mutable_position()->set_i_gain(_i[joint_index]);
    j_cmd.mutable_position()->set_d_gain(_d[joint_index]);
    j_cmd.mutable_velocity()->set_p_gain(0.0);
    j_cmd.mutable_velocity()->set_i_gain(0);
    j_cmd.mutable_velocity()->set_d_gain(0);
}

bool GazeboYarpControlBoardDriver::sendVelocitiesToGazebo(yarp::sig::Vector& refs) //NOT TESTED
{
    for (unsigned int j=0; j<_robot_number_of_joints; j++)
    {
        sendVelocityToGazebo(j,refs[j]);
    }
    return true;
}

bool GazeboYarpControlBoardDriver::sendVelocityToGazebo(int j,double ref) //NOT TESTED
{      
    /* SetVelocity method */
    /*gazebo::physics::JointPtr joint =  this->_robot->GetJoint(joint_names[j]);
    joint->SetMaxForce(0, joint->GetEffortLimit(0)*1.1); //<-- MAGIC NUMBER!!!!
    //      std::cout<<"MaxForce:" <<joint->GetMaxForce(0)<<std::endl;
    joint->SetVelocity(0,toRad(ref));
    */
    /* JointController method. If you pick this control method for control
     *      of joint velocities, you should also take care of the switching logic
     *      in setVelocityMode, setTorqueMode and setPositionMode:
     *      that is, the SetMarxForce(0,0) and SetVelocity(0,0) are no longer
     *      needed, but the JointController::AddJoint() method needs to be called
     *      when you switch to velocity mode, to make sure the PIDs get reset */
           gazebo::msgs::JointCmd j_cmd;
           prepareJointVelocityMsg(j_cmd,j,ref);
           jointCmdPub->WaitForConnection();
           jointCmdPub->Publish(j_cmd);
    /**/
    return true;
}

void GazeboYarpControlBoardDriver::prepareJointVelocityMsg(gazebo::msgs::JointCmd& j_cmd, const int j, const double ref) //NOT TESTED
{
    j_cmd.set_name(this->_robot->GetJoint(joint_names[j])->GetScopedName());
    j_cmd.mutable_position()->set_p_gain(0.0);
    j_cmd.mutable_position()->set_i_gain(0.0);
    j_cmd.mutable_position()->set_d_gain(0.0);
    j_cmd.mutable_velocity()->set_p_gain(0.200);
    j_cmd.mutable_velocity()->set_i_gain(0.02);
    j_cmd.mutable_velocity()->set_d_gain(0);
    j_cmd.mutable_velocity()->set_target(toRad(ref));
}

bool GazeboYarpControlBoardDriver::sendTorquesToGazebo(yarp::sig::Vector& refs) //NOT TESTED
{
    for (unsigned int j=0; j<_robot_number_of_joints; j++)
    {
        sendTorqueToGazebo(j,refs[j]);
    }
    return true;
}

bool GazeboYarpControlBoardDriver::sendTorqueToGazebo(const int j,const double ref) //NOT TESTED
{
    gazebo::msgs::JointCmd j_cmd;
    prepareJointTorqueMsg(j_cmd,j,ref);
    jointCmdPub->WaitForConnection();
    jointCmdPub->Publish(j_cmd);
    return true;
}

void GazeboYarpControlBoardDriver::prepareJointTorqueMsg(gazebo::msgs::JointCmd& j_cmd, const int j, const double ref) //NOT TESTED
{
    j_cmd.set_name(this->_robot->GetJoint(joint_names[j])->GetScopedName());
    j_cmd.mutable_position()->set_p_gain(0.0);
    j_cmd.mutable_position()->set_i_gain(0.0);
    j_cmd.mutable_position()->set_d_gain(0.0);
    j_cmd.mutable_velocity()->set_p_gain(0.0);
    j_cmd.mutable_velocity()->set_i_gain(0.0);
    j_cmd.mutable_velocity()->set_d_gain(0.0);
    j_cmd.set_force(ref);
}
