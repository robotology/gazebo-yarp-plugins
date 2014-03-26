/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include <gazebo_yarp_plugins/ControlBoardDriver.h>

//#include "../test/jointlogger.hpp"

#include <yarp/sig/all.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/os/all.h>
#include <stdio.h>

#define toDeg(X) (X*180.0/M_PI)

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::sig::file;
using namespace yarp::dev;

GazeboYarpControlBoardDriver::GazeboYarpControlBoardDriver(): RateThread(10)
{}
    
GazeboYarpControlBoardDriver::~GazeboYarpControlBoardDriver() {}

bool GazeboYarpControlBoardDriver::gazebo_init()
{
    //_robot = gazebo_pointer_wrapper::getModel();
    // std::cout<<"if this message is the last one you read, _robot has not been set"<<std::endl;
    //assert is a NOP in release mode. We should change the error handling either with an exception or something else
    assert ( _robot );
    if (!_robot) return false;

    std::cout<<"Robot Name: "<<_robot->GetName() <<std::endl;
    std::cout<<"# Joints: "<<_robot->GetJoints().size() <<std::endl;
    std::cout<<"# Links: "<<_robot->GetLinks().size() <<std::endl;

    this->robot_refresh_period=this->_robot->GetWorld()->GetPhysicsEngine()->GetUpdatePeriod() *1000.0;
    if( !setJointNames() ) return false;

    _controlboard_number_of_joints = joint_names.size();
    //pos_lock.unlock();
    pos.resize ( _controlboard_number_of_joints );
    zero_pos.resize ( _controlboard_number_of_joints );
    vel.resize ( _controlboard_number_of_joints );
    speed.resize ( _controlboard_number_of_joints );
    acc.resize ( _controlboard_number_of_joints );
    amp.resize ( _controlboard_number_of_joints );
    torque.resize ( _controlboard_number_of_joints ); torque.zero();
    ref_speed.resize ( _controlboard_number_of_joints );
    des_pos.resize ( _controlboard_number_of_joints );
    ref_pos.resize ( _controlboard_number_of_joints );
    ref_acc.resize ( _controlboard_number_of_joints );
    ref_torque.resize ( _controlboard_number_of_joints );
    max_pos.resize ( _controlboard_number_of_joints );
    min_pos.size ( _controlboard_number_of_joints );
    _positionPIDs.reserve ( _controlboard_number_of_joints );
    _velocityPIDs.reserve ( _controlboard_number_of_joints );
    _impedancePosPDs.reserve ( _controlboard_number_of_joints );
    torq_offset.resize( _controlboard_number_of_joints );
    min_stiffness.resize( _controlboard_number_of_joints, 0.0);
    max_stiffness.resize( _controlboard_number_of_joints, 1000.0);
    min_damping.resize( _controlboard_number_of_joints, 0.0);
    max_damping.resize( _controlboard_number_of_joints, 100.0);

    setMinMaxPos();
    setMinMaxImpedance();
    setPIDs();
    pos = 0;
    zero_pos=0;
    vel = 0;
    speed = 0;
    ref_speed=0;
    des_pos=0;
    ref_pos=0;
    ref_acc=0;
    ref_torque=0;
    acc = 0;
    amp = 1; // initially on - ok for simulator
    started=false;
    control_mode=new int[_controlboard_number_of_joints];
    motion_done=new bool[_controlboard_number_of_joints];
    _clock=0;
    torq_offset = 0;
    for ( unsigned int j=0; j<_controlboard_number_of_joints; ++j )
        control_mode[j]=VOCAB_CM_POSITION;

    std::cout << "gazebo_init set pid done!" << std::endl;

    this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin (
                                 boost::bind ( &GazeboYarpControlBoardDriver::onUpdate, this, _1 ) );
    
    gazebo_node_ptr = gazebo::transport::NodePtr ( new gazebo::transport::Node );
    gazebo_node_ptr->Init ( this->_robot->GetWorld()->GetName() );
    jointCmdPub = gazebo_node_ptr->Advertise<gazebo::msgs::JointCmd>
                  ( std::string ( "~/" ) + this->_robot->GetName() + "/joint_cmd" );

    _T_controller = 1;

    std::stringstream ss(plugin_parameters.find("initialConfiguration").toString());
    if(!(plugin_parameters.find("initialConfiguration") == ""))
    {
        double tmp = 0.0;
        yarp::sig::Vector initial_config(_controlboard_number_of_joints);
        unsigned int counter = 1;
        while(ss>>tmp)
        {
            if(counter > _controlboard_number_of_joints)
            {
                std::cout<<"To many element in initial configuration, stopping at element "<<counter<<std::endl;
                break;
            }
            initial_config[counter-1] = tmp;
            ref_pos[counter-1] = toDeg(tmp);
            des_pos[counter-1] = toDeg(tmp);
            pos[counter-1] = toDeg(tmp);
            counter++;
        }
        std::cout<<"INITIAL CONFIGURATION IS: "<<initial_config.toString()<<std::endl;

        for(unsigned int i = 0; i < _controlboard_number_of_joints; ++i)
        {
            gazebo::math::Angle a;
            a.SetFromRadian(initial_config[i]);
            std::string joint_name = joint_names[i];
            _robot->GetJoint(joint_name)->SetAngle(0,a);
        }
    }
    return true;
}

void GazeboYarpControlBoardDriver::compute_trj(const int j)
{
    if ( ( des_pos[j]-ref_pos[j] ) < -ROBOT_POSITION_TOLERANCE )
    {
        if ( ref_speed[j]!=0 ) des_pos[j]=des_pos[j]+ ( ref_speed[j]/1000.0 ) *robot_refresh_period* ( double ) _T_controller;
        motion_done[j]=false;
    }
    else if ( ( des_pos[j]-ref_pos[j] ) >ROBOT_POSITION_TOLERANCE )
    {
        if ( ref_speed[j]!=0 ) des_pos[j]=des_pos[j]- ( ref_speed[j]/1000.0 ) *robot_refresh_period* ( double ) _T_controller;
        motion_done[j]=false;
    }
    else
    {
        des_pos[j]=ref_pos[j];
        motion_done[j]=true;
    }
}

void GazeboYarpControlBoardDriver::onUpdate ( const gazebo::common::UpdateInfo & /*_info*/ )
{
    _clock++;

    if ( !started ) //This is a simple way to start with the robot in standing position
    {
        started=true;
        for ( unsigned int j = 0; j < _controlboard_number_of_joints; ++j )
            sendPositionToGazebo ( j, pos[j] );
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
    
    //logger.log(speed[2]);
    
    for ( unsigned int j=0; j<_controlboard_number_of_joints; ++j )
    {
        if ( control_mode[j]==VOCAB_CM_POSITION ) //set pos joint value, set vel joint value
        {   
            if ( _clock%_T_controller==0 )
            {
                compute_trj(j);
                //std::cout<<"pos: "<<pos[j]<<" ref_pos: "<<ref_pos[j]<<" ref_speed: "<<ref_speed[j]<<" period: "<<robot_refresh_period<<" result: "<<des_pos[j]<<std::endl;
                sendPositionToGazebo ( j,des_pos[j] );
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
        else if ( control_mode[j] == VOCAB_CM_IMPEDANCE_POS)
        {
            if ( _clock%_T_controller==0 )
            {
                compute_trj(j);
                sendImpPositionToGazebo ( j,des_pos[j] );
            }
        }
    }
}

void GazeboYarpControlBoardDriver::setMinMaxPos()  //NOT TESTED
{
    std::cout<<"Joint Limits"<<std::endl;
    for(unsigned int i = 0; i < _controlboard_number_of_joints; ++i)
    {
        max_pos[i] = this->_robot->GetJoint(joint_names[i])->GetUpperLimit(0).Degree();
        min_pos[i] = this->_robot->GetJoint(joint_names[i])->GetLowerLimit(0).Degree();
        std::cout<<joint_names[i]<<" max_pos: "<<max_pos[i]<<" min_pos: "<<min_pos[i]<<std::endl;
    }
}

bool hasEnding (std::string const &fullString, std::string const &ending)
{
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

bool GazeboYarpControlBoardDriver::setJointNames()  //WORKS
{
    std::cout << ".ini file found, using joint names in ini file" << std::endl;
    yarp::os::Bottle joint_names_bottle = plugin_parameters.findGroup("jointNames");

    if(joint_names_bottle.isNull()) {
        std::cout << "GazeboYarpControlBoardDriver::setJointNames(): Error cannot find jointNames." << std::endl;
        return false;
    }
    
    int nr_of_joints = joint_names_bottle.size()-1;
        
    joint_names.resize(nr_of_joints);
    
    const gazebo::physics::Joint_V & gazebo_models_joints = _robot->GetJoints();

    for(unsigned int i=0; i < joint_names.size(); i++ ) {
        bool joint_found = false;
        std::string controlboard_joint_name(joint_names_bottle.get(i+1).asString().c_str());
        
        for(unsigned int gazebo_joint = 0; gazebo_joint < gazebo_models_joints.size() && !joint_found; gazebo_joint++ ) {
            std::string gazebo_joint_name = gazebo_models_joints[gazebo_joint]->GetName();
            if( hasEnding(gazebo_joint_name,controlboard_joint_name) ) {
                joint_found = true;
                joint_names[i] = gazebo_joint_name;
            }
        }
        
        if( !joint_found ) { 
            std::cout << "GazeboYarpControlBoardDriver::setJointNames(): Error, cannot find joint " << joint_names[i] << std::endl;
            joint_names.resize(0);
            return false;
        }
       
    }     
    return true;
}

void GazeboYarpControlBoardDriver::setPIDsForGroup(std::string pidGroupName,
                                                   std::vector<GazeboYarpControlBoardDriver::PID> &pids,
                                                   enum PIDFeedbackTerm pidTerms)
{
    yarp::os::Property prop;
    if(plugin_parameters.check(pidGroupName.c_str()))
    {
        std::cout<<"Found PID information in plugin parameters group " << pidGroupName << std::endl;
        
        for(unsigned int i = 0; i < _controlboard_number_of_joints; ++i)
        {
            std::stringstream property_name;
            property_name<<"Pid";
            property_name<<i;
            
            yarp::os::Bottle& pid = plugin_parameters.findGroup(pidGroupName.c_str()).findGroup(property_name.str().c_str());
            
            GazeboYarpControlBoardDriver::PID pidValue = {0, 0, 0, -1, -1};
            if (pidTerms & PIDFeedbackTermProportionalTerm)
                pidValue.p = pid.get(1).asDouble();
            if (pidTerms & PIDFeedbackTermDerivativeTerm)
                pidValue.d = pid.get(2).asDouble();
            if (pidTerms & PIDFeedbackTermIntegrativeTerm)
                pidValue.i = pid.get(3).asDouble();
            
            pidValue.maxInt = pid.get(4).asDouble();
            pidValue.maxOut = pid.get(5).asDouble();
            
            
            pids.push_back(pidValue);
            std::cout<<"  P: "<<pidValue.p<<" I: "<<pidValue.i<<" D: "<<pidValue.d<<" maxInt: "<<pidValue.maxInt<<" maxOut: "<<pidValue.maxOut<<std::endl;
        }
        std::cout<<"OK!"<<std::endl;
    }
    else
    {
        double default_p = pidTerms & PIDFeedbackTermProportionalTerm ? 500.0 : 0;
        double default_i = pidTerms & PIDFeedbackTermIntegrativeTerm ? 0.1 : 0;
        double default_d = pidTerms & PIDFeedbackTermDerivativeTerm ? 1.0 : 0;
        std::cout<<"PID gain information not found in plugin parameters, using default gains ( "
        <<"P " << default_p << " I " << default_i << " D " << default_d << " )" <<std::endl;
        for(unsigned int i = 0; i < _controlboard_number_of_joints; ++i)
        {
            GazeboYarpControlBoardDriver::PID pid = {500, 0.1, 1.0, -1, -1};
            pids.push_back(pid);
        }
    }
}

void GazeboYarpControlBoardDriver::setMinMaxImpedance()
{

    yarp::os::Bottle& name_bot = plugin_parameters.findGroup("WRAPPER").findGroup("networks");
    std::string name = name_bot.get(1).toString();

    yarp::os::Bottle& kin_chain_bot = plugin_parameters.findGroup(name);
    if(kin_chain_bot.check("min_stiffness"))
    {
        std::cout<<"min_stiffness param found!"<<std::endl;
        yarp::os::Bottle& min_stiff_bot = kin_chain_bot.findGroup("min_stiffness");
        if(min_stiff_bot.size()-1 == _controlboard_number_of_joints)
        {
            for(unsigned int i = 0; i < _controlboard_number_of_joints; ++i)
                min_stiffness[i] = min_stiff_bot.get(i+1).asDouble();
        }
        else
            std::cout<<"Invalid number of params"<<std::endl;
    }
    else
        std::cout<<"No minimum stiffness value found in ini file, default one will be used!"<<std::endl;

    if(kin_chain_bot.check("max_stiffness"))
    {
        std::cout<<"max_stiffness param found!"<<std::endl;
        yarp::os::Bottle& max_stiff_bot = kin_chain_bot.findGroup("max_stiffness");
        if(max_stiff_bot.size()-1 == _controlboard_number_of_joints)
        {
            for(unsigned int i = 0; i < _controlboard_number_of_joints; ++i)
                max_stiffness[i] = max_stiff_bot.get(i+1).asDouble();
        }
        else
            std::cout<<"Invalid number of params"<<std::endl;
    }
    else
        std::cout<<"No maximum stiffness value found in ini file, default one will be used!"<<std::endl;

    if(kin_chain_bot.check("min_damping"))
    {
        std::cout<<"min_damping param found!"<<std::endl;
        yarp::os::Bottle& min_damping_bot = kin_chain_bot.findGroup("min_damping");
        if(min_damping_bot.size()-1 == _controlboard_number_of_joints)
        {
            for(unsigned int i = 0; i < _controlboard_number_of_joints; ++i)
                min_damping[i] = min_damping_bot.get(i+1).asDouble();
        }
        else
            std::cout<<"Invalid number of params"<<std::endl;
    }
    else
        std::cout<<"No minimum dampings value found in ini file, default one will be used!"<<std::endl;

    if(kin_chain_bot.check("max_damping"))
    {
        std::cout<<"max_damping param found!"<<std::endl;
        yarp::os::Bottle& max_damping_bot = kin_chain_bot.findGroup("max_damping");
        if(max_damping_bot.size()-1 == _controlboard_number_of_joints)
        {
            for(unsigned int i = 0; i < _controlboard_number_of_joints; ++i)
                max_damping[i] = max_damping_bot.get(i+1).asDouble();
        }
        else
            std::cout<<"Invalid number of params"<<std::endl;
    }
    else
        std::cout<<"No maximum damping value found in ini file, default one will be used!"<<std::endl;

    std::cout<<"min_stiffness: [ "<<min_stiffness.toString()<<" ]"<<std::endl;
    std::cout<<"max_stiffness: [ "<<max_stiffness.toString()<<" ]"<<std::endl;
    std::cout<<"min_damping: [ "<<min_damping.toString()<<" ]"<<std::endl;
    std::cout<<"max_damping: [ "<<max_damping.toString()<<" ]"<<std::endl;
}

void GazeboYarpControlBoardDriver::setPIDs()
{
    setPIDsForGroup("GAZEBO_PIDS", _positionPIDs, PIDFeedbackTermAllTerms);
    setPIDsForGroup("GAZEBO_VELOCITY_PIDS", _velocityPIDs, PIDFeedbackTerm(PIDFeedbackTermProportionalTerm | PIDFeedbackTermIntegrativeTerm));
    setPIDsForGroup("GAZEBO_IMPEDANCE_POSITION_PIDS", _impedancePosPDs, PIDFeedbackTerm(PIDFeedbackTermProportionalTerm | PIDFeedbackTermDerivativeTerm));
}

bool GazeboYarpControlBoardDriver::sendPositionsToGazebo(Vector &refs)
{
    for (unsigned int j=0; j<_controlboard_number_of_joints; j++)
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
    GazeboYarpControlBoardDriver::PID positionPID = _positionPIDs[joint_index];
    
    j_cmd.set_name(this->_robot->GetJoint(joint_names[joint_index])->GetScopedName());
    j_cmd.mutable_position()->set_target(toRad(ref));
    j_cmd.mutable_position()->set_p_gain(positionPID.p);
    j_cmd.mutable_position()->set_i_gain(positionPID.i);
    j_cmd.mutable_position()->set_d_gain(positionPID.d);
//     if (positionPID.maxInt > 0) {
//         j_cmd.mutable_position()->set_i_max(positionPID.maxInt);
//         j_cmd.mutable_position()->set_i_min(-positionPID.maxInt);
//     }
//     if (positionPID.maxOut > 0) {
//         j_cmd.mutable_position()->set_limit(positionPID.maxOut);
//     }
    j_cmd.mutable_velocity()->set_p_gain(0.0);
    j_cmd.mutable_velocity()->set_i_gain(0.0);
    j_cmd.mutable_velocity()->set_d_gain(0.0);
}

bool GazeboYarpControlBoardDriver::sendVelocitiesToGazebo(yarp::sig::Vector& refs) //NOT TESTED
{
    for (unsigned int j=0; j<_controlboard_number_of_joints; j++)
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
    GazeboYarpControlBoardDriver::PID velocityPID = _velocityPIDs[j];
    
    j_cmd.set_name(this->_robot->GetJoint(joint_names[j])->GetScopedName());
    j_cmd.mutable_position()->set_p_gain(0.0);
    j_cmd.mutable_position()->set_i_gain(0.0);
    j_cmd.mutable_position()->set_d_gain(0.0);
    j_cmd.mutable_velocity()->set_p_gain(velocityPID.p);
    j_cmd.mutable_velocity()->set_i_gain(velocityPID.i);
    j_cmd.mutable_velocity()->set_d_gain(velocityPID.d);
//     if (velocityPID.maxInt > 0) {
//         j_cmd.mutable_velocity()->set_i_max(velocityPID.maxInt);
//         j_cmd.mutable_velocity()->set_i_min(-velocityPID.maxInt);
//     }
//     if (velocityPID.maxOut > 0) {
//         j_cmd.mutable_velocity()->set_limit(velocityPID.maxOut);
//     }

    j_cmd.mutable_velocity()->set_target(toRad(ref));
}

bool GazeboYarpControlBoardDriver::sendTorquesToGazebo(yarp::sig::Vector& refs) //NOT TESTED
{
    for (unsigned int j=0; j<_controlboard_number_of_joints; j++)
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

void GazeboYarpControlBoardDriver::sendImpPositionToGazebo ( const int j, const double des )
{
    if(j >= 0 && j < _controlboard_number_of_joints)
    {
        /*
            Here joint positions and speeds are in [deg] and [deg/sec].
            Therefore also stiffness and damping has to be [Nm/deg] and [Nm*sec/deg].
            This is really unusual, btw, conversion factor is 180.0/pi.
        */
        double q = pos[j]-zero_pos[j];
        double t_ref = -_impedancePosPDs[j].p * (q - des) -_impedancePosPDs[j].d * speed[j] + torq_offset[j];
        sendTorqueToGazebo(j,t_ref);
    }
}

void GazeboYarpControlBoardDriver::sendImpPositionsToGazebo ( Vector &dess )
{
    for(unsigned int i = 0; i < _controlboard_number_of_joints; ++i)
        sendImpPositionToGazebo(i, dess[i]);
}
