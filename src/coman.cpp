/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia iCub Facility & ADVR
 * Authors: Lorenzo Natale and Paul Fitzpatrick and Enrico Mingo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
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
    return true;
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
        ref_speed.size(_robot_number_of_joints);
        ref_pos.size(_robot_number_of_joints);
        ref_acc.size(_robot_number_of_joints);
        max_pos.resize(_robot_number_of_joints);
        min_pos.size(_robot_number_of_joints);
        joint_names.reserve(_robot_number_of_joints);
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
      
      


      
}

/*
bool coman::threadInit()
{
    printf("FakeBot thread started\n");
    return true;
}

void coman::threadRelease()
{
    printf("FakeBot thread closed\n");
}

void coman::run()
{
//    static int count=0;
//    count++;
//    if (count%100==0)
//    {
//        printf("======\nfakebot thread looped [%d] times\n", count);
//        printf("Current position: %s\n", pos.toString().c_str());
//        printf("Current reference position: %s\n", ref_pos.toString().c_str());
//        printf("Reference speed: %s\n", ref_speed.toString().c_str());
//    }

    for(int j=0; j<_robot_number_of_joints; ++j)
    {
        // handle position mode
        if (control_mode[j]==VOCAB_CM_POSITION)
        {
            if ( (pos[j]-ref_pos[j]) < -ROBOT_POSITION_TOLERANCE)
            {
                pos[j]=pos[j]+ref_speed[j]*RateThread::getRate()/1000.0;
                motion_done[j]=false;
            }
            else if ( (pos[j]-ref_pos[j]) >ROBOT_POSITION_TOLERANCE)
            {
                pos[j]=pos[j]-ref_speed[j]*RateThread::getRate()/1000.0;
                motion_done[j]=false;
            }
            else
                motion_done[j]=true;
        }

        pos[j]=pos[j]+Random::normal(0,0.01);
    }
}
*/