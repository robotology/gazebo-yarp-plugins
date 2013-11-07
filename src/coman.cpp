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

bool coman::open(yarp::os::Searchable& config) 
{
    //if there is a .ini file, directly load it avoiding copyng all the data from config
    if( config.check("gazebo_ini_file_path") ) {
        plugin_parameters.fromConfigFile(config.find("gazebo_ini_file_path").asString().c_str());
    }
  
    //I love everything and every interface
    uintptr_t temp;
    std::istringstream iss(config.find("loving_gazebo_pointer").asString().c_str());
    boost::archive::text_iarchive archive(iss);
    try{
        archive>>temp;
    }
    catch (const char *ex)
    {
    //TODO
    }
    _robot=reinterpret_cast<gazebo::physics::Model*>(temp);
  
    gazebo_init();
    return RateThread::start();
}

void coman::gazebo_init()
{
    //_robot = gazebo_pointer_wrapper::getModel();
    std::cout<<"if this message is the last one you read, _robot has not been set"<<std::endl;
    assert(_robot);
  
    std::cout<<"Robot Name: "<<_robot->GetName()<<std::endl;
    std::cout<<"# Joints: "<<_robot->GetJoints().size()<<std::endl;
    std::cout<<"# Links: "<<_robot->GetLinks().size()<<std::endl;
  
        
        
    this->robot_refresh_period=this->_robot->GetWorld()->GetPhysicsEngine()->GetUpdatePeriod()*1000.0;
    setJointNames();

	_robot_number_of_joints = joint_names.size();
    pos_lock.unlock();
    pos.size(_robot_number_of_joints);
    zero_pos.size(_robot_number_of_joints);
    vel.size(_robot_number_of_joints);
    speed.size(_robot_number_of_joints);
    acc.size(_robot_number_of_joints);
    amp.size(_robot_number_of_joints);
    torque.size(_robot_number_of_joints);
    ref_speed.size(_robot_number_of_joints);
    ref_pos.size(_robot_number_of_joints);
    ref_acc.size(_robot_number_of_joints);
	ref_torque.size(_robot_number_of_joints);
    max_pos.resize(_robot_number_of_joints);
    min_pos.size(_robot_number_of_joints);
    _p.reserve(_robot_number_of_joints);
    _i.reserve(_robot_number_of_joints);
    _d.reserve(_robot_number_of_joints);
	
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
    for(int j=0; j<_robot_number_of_joints; ++j)
        control_mode[j]=VOCAB_CM_POSITION;

	this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
                                     boost::bind(&coman::onUpdate, this, _1));
    gazebo_node_ptr = gazebo::transport::NodePtr(new gazebo::transport::Node);
    gazebo_node_ptr->Init(this->_robot->GetWorld()->GetName());
    jointCmdPub = gazebo_node_ptr->Advertise<gazebo::msgs::JointCmd>
        (std::string("~/") + this->_robot->GetName() + "/joint_cmd");
      
      
    _T_controller = 10;


}

//We need a thread to publish some extra information like joint torques and velocities.
bool coman::threadInit()
{
    std::string gazebo_group_name = "GAZEBO";
    std::stringstream property_name;
    property_name<<"name";
    yarp::os::Bottle& name = plugin_parameters.findGroup(gazebo_group_name.c_str()).findGroup(property_name.str().c_str());


    std::stringstream port_name_torque;
    port_name_torque<<"/wholeBodyDynamics"<<name.get(1).asString().c_str()<<"/Torques:o";
    _joint_torq_port.open(port_name_torque.str().c_str());
    std::stringstream port_name_speed;
    port_name_speed<<"/wholeBodyDynamics"<<name.get(1).asString().c_str()<<"/Speeds:o";
    _joint_speed_port.open(port_name_speed.str().c_str());
    return true;
}

void coman::afterStart(bool s)
{
    if(s)
        printf("TorqueAndSpeedPublisher started successfully\n");
    else
        printf("TorqueAndSpeedPublisher did not start\n");
}

void coman::run()
{
    yarp::os::Bottle bot1;
    yarp::os::Bottle bot2;
    for(unsigned int j = 0; j < _robot_number_of_joints; ++j){
        bot1.addDouble(torque[j]);
        bot2.addDouble(speed[j]);
    }
    _joint_torq_port.write(bot1);
    _joint_speed_port.write(bot2);
    bot1.clear();
    bot2.clear();
}

void coman::threadRelease()
{
    printf("Goodbye from TorqueAndSpeedPublisher\n");
}
