/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia iCub Facility & ADVR
 * Authors: Lorenzo Natale and Paul Fitzpatrick and Enrico Mingo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef COMAN_H
#define COMAN_H

#include <yarp/sig/all.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/os/all.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo_pointer_wrapper.h"
#include <gazebo/transport/transport.hh>
#include <mutex>

#define toRad(X) (X*M_PI/180.0)
const double ROBOT_POSITION_TOLERANCE=0.9;


static const std::string pid_config_abs_path = "../config/pid.ini";

namespace yarp {
    namespace dev {
      class coman;
    }
}

class yarp::dev::coman : public DeviceDriver,
            public IPositionControl,
            public IVelocityControl,
            public IAmplifierControl,
            public IEncoders,
            public IControlCalibration2,
            public IControlLimits,
            public DeviceResponder,
            public IControlMode
            //,private yarp::os::RateThread
{
public:
    coman()//:RateThread(0)
    {
        _robot = gazebo_pointer_wrapper::getModel();
	this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
          boost::bind(&coman::onUpdate, this, _1));
	std::cout<<"Robot Name: "<<_robot->GetName()<<std::endl;
        std::cout<<"# Joints: "<<_robot->GetJoints().size()<<std::endl;
        std::cout<<"# Links: "<<_robot->GetLinks().size()<<std::endl;
	this->robot_refresh_period=this->_robot->GetWorld()->GetPhysicsEngine()->GetUpdatePeriod()*1000.0;
	gazebo_node_ptr = gazebo::transport::NodePtr(new gazebo::transport::Node);
        gazebo_node_ptr->Init(this->_robot->GetWorld()->GetName());
        jointCmdPub = gazebo_node_ptr->Advertise<gazebo::msgs::JointCmd>
                (std::string("~/") + this->_robot->GetName() + "/joint_cmd");


        _robot_number_of_joints = _robot->GetJoints().size();
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


        setJointNames();
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

        for(int j=0; j<_robot_number_of_joints; ++j)
            control_mode[j]=VOCAB_CM_POSITION;
    }

    ~coman()
    {
        delete [] control_mode;
        delete [] motion_done;
    }

    void onUpdate(const gazebo::common::UpdateInfo & /*_info*/)
    {
      if (!started)
      {
	started=true;
	double temp=0;//[_robot_number_of_joints];
	for (int j=0;j<_robot_number_of_joints;j++)
	{//	  temp[j]=0;
//	positionMove(temp);
	sendPositionToGazebo(j,temp);
	}
      }
      pos_lock.lock();
      // read sensors (for now only joints angle)
	auto joints=_robot->GetJoints();
	int j=0;
	for (auto joint:joints)
	{
	  pos[j]=joint->GetAngle(0).Degree()-zero_pos[j];  //TODO: if zero_pos=0, it works, if zero_pos=pos[j], pos[j] return 0, if zero_pos=k, pos[j]return 0-k, but since it is an angle, you may get 2*pi
	  //std::cout<<"joint"<<j<<" pos"<<pos[j]<<std::endl;        
	  pos[j]=pos[j]+yarp::os::Random::normal(0,0.01);

	  j++;
	}
      pos_lock.unlock();
      // send positions to the actuators
      
      yarp::sig::Vector temp=ref_pos; //if VOCAB_CM_POSITION I will not do anything
       
       for(int j=0; j<_robot_number_of_joints; ++j)
    {
        // handle position mode
        if (control_mode[j]==VOCAB_CM_VELOCITY) //TODO check if VOCAB_CM_POSITION or VOCAB_CM_VELOCITY
        {
            if ( (pos[j]-ref_pos[j]) < -ROBOT_POSITION_TOLERANCE)
            {
                temp[j]=pos[j]+ref_speed[j]*robot_refresh_period/1000.0;
                motion_done[j]=false;
            }
            else if ( (pos[j]-ref_pos[j]) >ROBOT_POSITION_TOLERANCE)
            {
                temp[j]=pos[j]-ref_speed[j]*robot_refresh_period/1000.0;
                motion_done[j]=false;
            }
            else
                motion_done[j]=true;
        }
      
      

     }
      sendPositionsToGazebo(temp);
    }

    
    bool sendPositionsToGazebo(yarp::sig::Vector refs)
    {
       for (int j=0;j<_robot_number_of_joints;j++)
       {
	 sendPositionToGazebo(j,refs[j]);
       }
    }
    
    bool sendPositionToGazebo(int j,double ref)
    {
            gazebo::msgs::JointCmd j_cmd;
            j_cmd.set_name(this->_robot->GetJoint(joint_names[j])->GetScopedName()); //TODO maybe cache this values inside the class? e.g. set_name(_joint_scoped_names[j])
            j_cmd.mutable_position()->set_target(toRad(ref));
            j_cmd.mutable_position()->set_p_gain(500.0); //move somewhere else!
            jointCmdPub->WaitForConnection();
            jointCmdPub->Publish(j_cmd);
 
    }
    
    // thread
    /*virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();
    */
    virtual bool open(yarp::os::Searchable& config);

    /**
     * This is asyncronous, but do we care?
     */
    virtual bool positionMove(int j, double ref) {
        if (j<_robot_number_of_joints) {
            ref_pos[j] = ref; //we will use this ref_pos in the next simulation onUpdate call to ask gazebo to set PIDs ref_pos to this value
        }
        return true;
    }
    
    virtual bool getEncoder(int j, double *v) {
      std::cout<<"get encoder chiamata"<<std::endl;
      //pos_lock.lock();
        if (j<_robot_number_of_joints) {
            (*v) = pos[j];
        }
      //pos_lock.unlock();
        return true;
	
    }

    virtual bool getEncoders(double *encs) {
      //pos_lock.lock();
           //std::cout<<"get encoders chiamata"<<std::endl;
        for (int i=0; i<_robot_number_of_joints; ++i) {
            encs[i] = pos[i]; //should we just use memcopy here?
        }
        return true;
	//pos_lock.unlock();
    }

    
    // IPositionControl etc.

    virtual bool getAxes(int *ax) {
        *ax = _robot_number_of_joints;
        return true;
    }

    virtual bool setPositionMode() {
        return true;
    }

    

    virtual bool positionMove(const double *refs) {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            ref_pos[i] = refs[i];
            //positionMove(i,refs[i]);
        }
        return true;
    }

    virtual bool relativeMove(int j, double delta) {
        if (j<_robot_number_of_joints) {
            ref_pos[j] =pos[j] + delta; //TODO check if this is ok or ref_pos=ref_pos+delta!!!
        }
        return true;
    }


    virtual bool relativeMove(const double *deltas) {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            ref_pos[i] = pos[i]+ deltas[i]; //TODO check if this is ok or ref_pos=ref_pos+delta!!!
        }
        return true;
    }

    virtual bool checkMotionDone(int j, bool *flag) {
        *flag=motion_done[j];
        return true;
    }


    virtual bool checkMotionDone(bool *flag) {
        bool temp_flag=true;
	//*flag=true;
        for(int j=0; j<_robot_number_of_joints; ++j)
         {
	   //*flag&&motion_done[j]; //It's compiler job to make code unreadable and optimized, not programmer's 
	   temp_flag=temp_flag && motion_done[j];
	}
	*flag=temp_flag;
        return true;
    }


    virtual bool setRefSpeed(int j, double sp) {
        if (j<_robot_number_of_joints) {
            ref_speed[j] = sp;
        }
        return true;
    }

    virtual bool setRefSpeeds(const double *spds) {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            ref_speed[i] = spds[i];
        }
        return true;
    }


    virtual bool setRefAcceleration(int j, double acc) {
        if (j<_robot_number_of_joints) {
            ref_acc[j] = acc;
        }
        return true;
    }


    virtual bool setRefAccelerations(const double *accs) {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            ref_acc[i] = accs[i];
        }
        return true;
    }

    virtual bool getRefSpeed(int j, double *ref) {
        if (j<_robot_number_of_joints) {
            (*ref) = ref_speed[j];
        }
        return true;
    }


    virtual bool getRefSpeeds(double *spds) {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            spds[i] = ref_speed[i];
        }
        return true;
    }


    virtual bool getRefAcceleration(int j, double *acc) {
        if (j<_robot_number_of_joints) {
            (*acc) = ref_acc[j];
        }
        return true;
    }

    virtual bool getRefAccelerations(double *accs) {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            accs[i] = ref_acc[i];
        }
        return true;
    }


    virtual bool stop(int j) {
        ref_pos[j]=pos[j];
        return true;
    }


    virtual bool stop()
    {
        ref_pos=pos;
        return true;
    }


    virtual bool close() {
        return true;
    }

    /**
     * Since we don't know how to reset gazebo encoders, we will simply add the actual value to the future encoders readings
     */
    virtual bool resetEncoder(int j) {
        if (j<_robot_number_of_joints) {
            zero_pos[j] = pos[j];
         }
        return true;
    }

    virtual bool resetEncoders() {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            zero_pos[i] = pos[i];
        }
        return true;
    }

    virtual bool setEncoder(int j, double val) {
        if (j<_robot_number_of_joints) {
            zero_pos[j] = val-pos[j];
        }
        return true;
    }

    virtual bool setEncoders(const double *vals) {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            zero_pos[i] = vals[i]-pos[i];
        }
        return true;
    }

    
    virtual bool getEncoderSpeed(int j, double *sp) {
        if (j<_robot_number_of_joints) {
            (*sp) = 0;
        }
        return true;
    }

    virtual bool getEncoderSpeeds(double *spds) {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            spds[i] = 0;
        }
        return true;
    }

    virtual bool getEncoderAcceleration(int j, double *spds) {
        if (j<_robot_number_of_joints) {
            (*spds) = 0;
        }
        return true;
    }

    virtual bool getEncoderAccelerations(double *accs) {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            accs[i] = 0;
        }
        return true;
    }

    virtual bool velocityMove(int j, double sp) {
        if (j<_robot_number_of_joints) {
            vel[j] = sp;
        }
        return true;
    }

    virtual bool velocityMove(const double *sp) {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            vel[i] = sp[i];
        }
        return true;
    }

    virtual bool enableAmp(int j) {
        if (j<_robot_number_of_joints) {
            amp[j] = 1;
            control_mode[j]=VOCAB_CM_POSITION;
        }
        return true;
    }

    virtual bool disableAmp(int j) {
        if (j<_robot_number_of_joints) {
            amp[j] = 0;
            control_mode[j]=VOCAB_CM_IDLE;
        }
        return true;
    }

    virtual bool getCurrent(int j, double *val) {
        if (j<_robot_number_of_joints) {
            val[j] = amp[j];
        }
        return true;
    }

    virtual bool getCurrents(double *vals) {
        for (int i=0; i<_robot_number_of_joints; i++) {
            vals[i] = amp[i];
        }
        return true;
    }

    virtual bool setMaxCurrent(int j, double v) {
        return true;
    }

    virtual bool getAmpStatus(int *st) {
        *st = 0;
        return true;
    }

    virtual bool getAmpStatus(int k, int *v)
    {
        *v=0;
        return true;
    }

    virtual bool calibrate2(int j, unsigned int iv, double v1, double v2, double v3)
    {
        fprintf(stderr, "fakebot: calibrating joint %d with parameters %u %lf %lf %lf\n", j, iv, v1, v2, v3);
        return true;
    }

    virtual bool done(int j)
    {
        fprintf(stderr , "fakebot: calibration done on joint %d.\n", j);
        return true;
    }

    // IControlLimits
    virtual bool getLimits(int axis, double *min, double *max)
    {
        *min=min_pos[axis];
        *max=max_pos[axis];
        return true;
    }

    virtual bool setLimits(int axis, double min, double max)
    {
        max_pos[axis]=max;
        min_pos[axis]=min;
        return true;
    }


    // IControlMode
    virtual bool setPositionMode(int j){
      control_mode[j]=VOCAB_CM_POSITION;
      std::cout<<"control mode = position"<<j<<std::endl;
    }
    virtual bool setVelocityMode(int j){
      control_mode[j]=VOCAB_CM_VELOCITY;
      std::cout<<"control mode = speed"<<j<<std::endl;
    }

    virtual bool setVelocityMode()
    {
       for(int j=0; j<_robot_number_of_joints; j++)
           { 
	     control_mode[j]=VOCAB_CM_VELOCITY;
	    std::cout<<"control mode = speed for all joints"<<std::endl;
	  }
    }

    virtual bool setTorqueMode(int j){ return false; }
    virtual bool setImpedancePositionMode(int j){return false;}
    virtual bool setImpedanceVelocityMode(int j){return false;}
    virtual bool setOpenLoopMode(int j){return false; }
    virtual bool getControlMode(int j, int *mode){mode[j]=control_mode[j];}
    virtual bool getControlModes(int *modes)
    {
        for(int j=0;j<_robot_number_of_joints; ++j)
            {modes[j]=control_mode[j];}
        return true;
    }
/**/




private:
    unsigned int robot_refresh_period; //ms
    gazebo::physics::ModelPtr _robot;
    gazebo::event::ConnectionPtr updateConnection;
    unsigned int _robot_number_of_joints;

    
    
    /**
     * The GAZEBO position of each joints, readonly from outside this interface
     */
    yarp::sig::Vector pos;
    
    /**
     * The zero position is the position of the GAZEBO joint that will be read as the starting one
     * i.e. getEncoder(j)=zero_pos+gazebo.getEncoder(j);
     */
    yarp::sig::Vector zero_pos;
    
    yarp::sig::Vector vel, speed, acc, amp;
    std::mutex pos_lock;
    yarp::sig::Vector ref_speed, ref_pos, ref_acc;
    yarp::sig::Vector max_pos, min_pos;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> back, fore;

    std::vector<std::string> joint_names;
    gazebo::transport::NodePtr gazebo_node_ptr;
    gazebo::transport::PublisherPtr jointCmdPub;
    std::vector<double> _p;
    std::vector<double> _i;
    std::vector<double> _d;

    bool *motion_done;
    int  *control_mode;
    bool command_changed;
    bool started;

    void setMinMaxPos()
    {
        std::cout<<"Joint Limits"<<std::endl;
        gazebo::physics::Joint_V joints = _robot->GetJoints();
        for(unsigned int i = 0; i < _robot_number_of_joints; ++i)
        {
            gazebo::physics::JointPtr j = joints[i];
            max_pos[i] = j->GetUpperLimit(0).Degree();
            min_pos[i] = j->GetLowerLimit(0).Degree();
            std::cout<<joint_names[i]<<" max_pos: "<<max_pos[i]<<" min_pos: "<<min_pos[i]<<std::endl;
        }
    }

    void setJointNames()
    {
        gazebo::physics::Joint_V joints = _robot->GetJoints();
        for(unsigned int i = 0; i < _robot_number_of_joints; ++i)
        {
            gazebo::physics::JointPtr j = joints[i];
            joint_names.push_back(j->GetName());
        }
    }

    void setPIDs()
    {
        yarp::os::Property prop;
        if(prop.fromConfigFile(pid_config_abs_path.c_str()))
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

    void prepareJointMsg(gazebo::msgs::JointCmd& j_cmd, int joint_index, const double ref)
    {
        j_cmd.set_name(this->_robot->GetJoint(joint_names[joint_index])->GetScopedName());
        j_cmd.mutable_position()->set_target(toRad(ref));
        j_cmd.mutable_position()->set_p_gain(_p[joint_index]);
        j_cmd.mutable_position()->set_i_gain(_i[joint_index]);
        j_cmd.mutable_position()->set_d_gain(_d[joint_index]);
    }
};

#endif //COMAN_H
