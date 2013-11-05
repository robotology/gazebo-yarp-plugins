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
    public IControlMode,
    public ITorqueControl
    //,private yarp::os::RateThread
{
public:
    coman()//:RateThread(0)
    {
        //almost everything is done in the open() method
    }

    ~coman()
    {
    }

    /**
     * Gazebo stuff
     * 
     */
    
     void gazebo_init();
     
    
    void onUpdate(const gazebo::common::UpdateInfo & /*_info*/)
    {
        if (!started)
        {
            started=true;
            double temp=0;//[_robot_number_of_joints];
            for(unsigned int j=0; j<_robot_number_of_joints; j++)
                sendPositionToGazebo(j,temp);
        }

        pos_lock.lock();
        
        // Sensing position & torque
        for(int jnt_cnt=0; jnt_cnt < joint_names.size(); jnt_cnt++ )
        {
            /** \todo consider multi-dof joint ? */
            pos[jnt_cnt] = this->_robot->GetJoint(joint_names[jnt_cnt])->GetAngle(0).Degree();

//            if(_clock%(2*1000) == 0){
//                //if(joint_names[jnt_cnt] == "COMAN::RShSag" )//||
//                   //joint_names[jnt_cnt] == "COMAN::RShLat" ||
//                   //joint_names[jnt_cnt] == "COMAN::RShYaw" ||
//                   //joint_names[jnt_cnt] == "COMAN::RElbj")
//                {
//                gazebo::physics::JointWrench jnt_wrench = this->_robot->GetJoint(joint_names[jnt_cnt])->GetForceTorque(0);
//                gazebo::math::Vector3 jnt_torque1 = jnt_wrench.body1Torque;
//                std::cout<<"Joint "<<joint_names[jnt_cnt]<<" torque1: [ "<<jnt_torque1.x<<" "<<
//                           jnt_torque1.y<<" "<<jnt_torque1.z<<" ]"<<std::endl;
//                gazebo::math::Vector3 jnt_torque2 = jnt_wrench.body2Torque;
//                std::cout<<"Joint "<<joint_names[jnt_cnt]<<" torque2: [ "<<jnt_torque2.x<<" "<<
//                           jnt_torque2.y<<" "<<jnt_torque2.z<<" ]"<<std::endl;
//                gazebo::math::Vector3 jnt_local_axis = this->_robot->GetJoint(joint_names[jnt_cnt])->GetLocalAxis(0);
//                std::cout<<"Joint "<<joint_names[jnt_cnt]<<" jnt_local_axis: [ "<<jnt_local_axis.x<<" "<<
//                            jnt_local_axis.y<<" "<<jnt_local_axis.z<<" ]"<<std::endl;
//                std::cout<<std::endl;
//                std::cout<<std::endl;
//                }
//}
        }
        
        pos_lock.unlock();

        _clock++;

        for(unsigned int j=0; j<_robot_number_of_joints; ++j)
        {
            /*if (control_mode[j]==VOCAB_CM_POSITION)
            {
                sendPositionToGazebo(j,ref_pos[j]);
            }*/
	    
	    if (control_mode[j]==VOCAB_CM_POSITION) //set pos joint value, set vel joint value
	    {
		
            if (_clock%_T_controller==0)
            {
                double temp=ref_pos[j];
                if ( (pos[j]-ref_pos[j]) < -ROBOT_POSITION_TOLERANCE)
                {
                    if(ref_speed[j]!=0) temp=pos[j]+(ref_speed[j]/1000.0)*robot_refresh_period*(double)_T_controller;
                    motion_done[j]=false;
                }
                else if ( (pos[j]-ref_pos[j]) >ROBOT_POSITION_TOLERANCE)
                {
                    if(ref_speed[j]!=0) temp=pos[j]-(ref_speed[j]/1000.0)*robot_refresh_period*(double)_T_controller;
                    motion_done[j]=false;
                }
                else
                    motion_done[j]=true;
			
//            std::cout<<"pos: "<<pos[j]<<" ref_pos: "<<ref_pos[j]<<" ref_speed: "<<ref_speed[j]<<" period: "<<robot_refresh_period<<" result: "<<temp<<std::endl;
              sendPositionToGazebo(j,temp);
            }
        }
        else if(control_mode[j]==VOCAB_CM_VELOCITY) //set vmo joint value
	    {
            if (_clock%_T_controller==0)
            {
                sendVelocityToGazebo(j,vel[j]);
                //std::cout<<" velocity "<<vel[j]<<'('<<toRad(vel[j])<<')'<<" to joint "<<j<<std::endl;
            }
	    }
        else if(control_mode[j]==VOCAB_CM_TORQUE)
	    {
            if (_clock%_T_controller==0)
            {
                sendTorqueToGazebo(j,ref_torque[j]);
                //std::cout<<" torque "<<ref_torque[j]<<" to joint "<<j<<std::endl;
            }
	    }
	}  
    }
    
    virtual bool open(yarp::os::Searchable& config);

    /**
     * Yarp interfaces start here
     */
    
    /**
     * This is asyncronous, but do we care?
     */
    virtual bool positionMove(int j, double ref) //WORKS
    {
        if (j<_robot_number_of_joints) {
            ref_pos[j] = ref; //we will use this ref_pos in the next simulation onUpdate call to ask gazebo to set PIDs ref_pos to this value
        }
        return true;
    }

    virtual bool getEncoder(int j, double *v) //WORKS
    {
        std::cout<<"get encoder chiamata"<<std::endl;
        //pos_lock.lock();
        if (j<_robot_number_of_joints) {
            (*v) = pos[j]-zero_pos[j];
        }
        //pos_lock.unlock();
        return true;

    }

    virtual bool getEncoders(double *encs) //WORKS
    {
        //pos_lock.lock();
        for (int i=0; i<_robot_number_of_joints; ++i) {
            encs[i] = pos[i]-zero_pos[i];  //should we just use memcopy here?
        }
        return true;
        //pos_lock.unlock();
    }


    // IPositionControl etc.

    virtual bool getAxes(int *ax) // WORKS
    {
        *ax = _robot_number_of_joints;
        return true;
    }


    virtual bool positionMove(const double *refs) //WORKS
    {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            ref_pos[i] = refs[i];
        }
        return true;
    }

    virtual bool relativeMove(int j, double delta) //NOT TESTED
    {
        if (j<_robot_number_of_joints) {
            ref_pos[j] =pos[j] + delta; //TODO check if this is ok or ref_pos=ref_pos+delta!!!
        }
        return true;
    }


    virtual bool relativeMove(const double *deltas) //NOT TESTED
    {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            ref_pos[i] = pos[i]+ deltas[i]; //TODO check if this is ok or ref_pos=ref_pos+delta!!!
        }
        return true;
    }

    virtual bool checkMotionDone(int j, bool *flag) //NOT TESTED
    {
        *flag=motion_done[j];
        return true;
    }


    virtual bool checkMotionDone(bool *flag) //NOT TESTED
    {
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

    
    virtual bool setRefTorque(int j, double t) //NOT TESTED
    {
        std::cout<<std::endl<<"Joint"<<j<<" trq: "<<t<<std::endl<<std::endl;
        if (j<_robot_number_of_joints)
        {
            ref_torque[j] = t;
        }
        return true;
    }
    
    virtual bool setRefTorques(const double *t) //NOT TESTED
    {
        for (unsigned int i=0; i<_robot_number_of_joints; ++i)
            setRefTorque(i, t[i]);
        return true;
    }

    /// @arg sp [deg/sec]
    virtual bool setRefSpeed(int j, double sp) //WORKS
    {
        if (j<_robot_number_of_joints) {
            ref_speed[j] = sp;
        }
        return true;
    }

    /// @arg spds [deg/sec]
    virtual bool setRefSpeeds(const double *spds) //NOT TESTED
    {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            ref_speed[i] = spds[i];
        }
        return true;
    }


    virtual bool setRefAcceleration(int j, double acc) //NOT IMPLEMENTED
    {
        if (j<_robot_number_of_joints) {
            ref_acc[j] = acc;
        }
        return true;
    }


    virtual bool setRefAccelerations(const double *accs) //NOT IMPLEMENTED
    {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            ref_acc[i] = accs[i];
        }
        return true;
    }

    virtual bool getRefSpeed(int j, double *ref) //WORKS
    {
        if (j<_robot_number_of_joints) {
            (*ref) = ref_speed[j];
        }
        return true;
    }


    virtual bool getRefSpeeds(double *spds) //WORKS
    {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            spds[i] = ref_speed[i];
        }
        return true;
    }


    virtual bool getRefAcceleration(int j, double *acc) //NOT IMPLEMENTED
    {
        if (j<_robot_number_of_joints) {
            (*acc) = ref_acc[j];
        }
        return true;
    }

    virtual bool getRefAccelerations(double *accs) //NOT IMPLEMENTED
    {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            accs[i] = ref_acc[i];
        }
        return true;
    }


    virtual bool stop(int j) //WORKS
    {
        ref_pos[j]=pos[j];
        return true;
    }


    virtual bool stop() //WORKS
    {
        ref_pos=pos;
        return true;
    }


    virtual bool close() //NOT IMPLEMENTED
    {
        delete [] control_mode;
        delete [] motion_done;
        return true;
    }

    /**
     * Since we don't know how to reset gazebo encoders, we will simply add the actual value to the future encoders readings
     */
    virtual bool resetEncoder(int j) //WORKS
    {
        if (j<_robot_number_of_joints) {
            zero_pos[j] = pos[j];
        }
        return true;
    }

    virtual bool resetEncoders() //WORKS
    {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            zero_pos[i] = pos[i];
        }
        return true;
    }

    virtual bool setEncoder(int j, double val) //WORKS
    {
        if (j<_robot_number_of_joints) {
            zero_pos[j] = pos[j]-val;
        }
        return true;
    }

    virtual bool setEncoders(const double *vals) //WORKS
    {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            zero_pos[i] = pos[i]-vals[i];
        }
        return true;
    }


    virtual bool getEncoderSpeed(int j, double *sp) //NOT IMPLEMENTED
    {
        if (j<_robot_number_of_joints) {
            (*sp) = 0;
        }
        return true;
    }

    virtual bool getEncoderSpeeds(double *spds) //NOT IMPLEMENTED
    {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            spds[i] = 0;
        }
        return true;
    }

    virtual bool getEncoderAcceleration(int j, double *spds) //NOT IMPLEMENTED
    {
        if (j<_robot_number_of_joints) {
            (*spds) = 0;
        }
        return true;
    }

    virtual bool getEncoderAccelerations(double *accs) //NOT IMPLEMENTED
    {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            accs[i] = 0;
        }
        return true;
    }

    virtual bool velocityMove(int j, double sp) //NOT TESTED
    {
        if (j<_robot_number_of_joints) 
	{
            vel[j] = sp;
	    
        }
        return true;
    }

    virtual bool velocityMove(const double *sp) //NOT TESTED
    {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            vel[i] = sp[i];
        }
        return true;
    }

    virtual bool enableAmp(int j) //NOT IMPLEMENTED
    {
        if (j<_robot_number_of_joints) {
            amp[j] = 1;
            control_mode[j]=VOCAB_CM_POSITION;
        }
        return true;
    }

    virtual bool disableAmp(int j) //NOT IMPLEMENTED
    {
        if (j<_robot_number_of_joints) {
            amp[j] = 0;
            control_mode[j]=VOCAB_CM_IDLE;
        }
        return true;
    }

    virtual bool getCurrent(int j, double *val) //NOT IMPLEMENTED
    {
        if (j<_robot_number_of_joints) {
            val[j] = amp[j];
        }
        return true;
    }

    virtual bool getCurrents(double *vals) //NOT IMPLEMENTED
    {
        for (int i=0; i<_robot_number_of_joints; i++) {
            vals[i] = amp[i];
        }
        return true;
    }

    virtual bool setMaxCurrent(int j, double v) //NOT IMPLEMENTED
    {
        return true;
    }

    virtual bool getAmpStatus(int *st) //NOT IMPLEMENTED
    {
        *st = 0;
        return true;
    }

    virtual bool getAmpStatus(int k, int *v) //NOT IMPLEMENTED
    {
        *v=0;
        return true;
    }

    virtual bool calibrate2(int j, unsigned int iv, double v1, double v2, double v3) //NOT IMPLEMENTED
    {
        fprintf(stderr, "fakebot: calibrating joint %d with parameters %u %lf %lf %lf\n", j, iv, v1, v2, v3);
        return true;
    }

    virtual bool done(int j) // NOT IMPLEMENTED
    {
        fprintf(stderr , "fakebot: calibration done on joint %d.\n", j);
        return true;
    }

    // IControlLimits
    virtual bool getLimits(int axis, double *min, double *max) //NOT TESTED
    {
        *min=min_pos[axis];
        *max=max_pos[axis];
        return true;
    }

    virtual bool setLimits(int axis, double min, double max) //NOT TESTED
    {
        max_pos[axis]=max;
        min_pos[axis]=min;
        return true;
    }


    // IControlMode
    virtual bool setPositionMode(int j) //WORKS
    {
        control_mode[j]=VOCAB_CM_POSITION;
        std::cout<<"control mode = position "<<j<<std::endl;
    }
    
    virtual bool setPositionMode() //NOT TESTED
    {
        for(int j=0; j<_robot_number_of_joints; j++)
        {
            control_mode[j]=VOCAB_CM_POSITION;
            std::cout<<"control mode = position for all joints"<<std::endl;
        }
    }
    
    virtual bool setVelocityMode(int j) //WORKS
    {
        control_mode[j]=VOCAB_CM_VELOCITY;
        std::cout<<"control mode = speed "<<j<<std::endl;
    }

    virtual bool setVelocityMode() //NOT TESTED
    {
        for(int j=0; j<_robot_number_of_joints; j++)
        {
            control_mode[j]=VOCAB_CM_VELOCITY;
            std::cout<<"control mode = speed for all joints"<<std::endl;
        }
    }

    virtual bool setTorqueMode(int j) //NOT TESTED
    {
        control_mode[j]=VOCAB_CM_TORQUE;
        std::cout<<"control mode = torque "<<j<<std::endl;
    }
 
    virtual bool setTorqueMode() //NOT TESTED
    {
        for(int j=0; j<_robot_number_of_joints; j++)
        {
            control_mode[j]=VOCAB_CM_TORQUE;
            std::cout<<"control mode = torque for all joints"<<std::endl;
        }
    }
    virtual bool setImpedancePositionMode(int j)//NOT IMPLEMENTED
    {
        return false;
    }
    virtual bool setImpedanceVelocityMode(int j) //NOT IMPLEMENTED
    {
        return false;
    }
    virtual bool setOpenLoopMode(int j) //NOT IMPLEMENTED
    {
        return false;
    }
    virtual bool getControlMode(int j, int *mode) //WORKS
    {
        mode[j]=control_mode[j];
    }
    virtual bool getControlModes(int *modes)
    {
        for(int j=0; j<_robot_number_of_joints; ++j)
        {
            modes[j]=control_mode[j];
        }
        return true;
    }
   
    /**/
    
    virtual bool getRefTorque(int j, double *t)
    {
        if (j<_robot_number_of_joints) {
            t[j] = ref_torque[j];
        }
        return true;
    } //NOT TESTED

       
    virtual bool getRefTorques(double *t)
    {
        for(unsigned int i = 0; i < _robot_number_of_joints; ++i)
            getRefTorque(i, t);
        return true;
    } //NOT TESTED

    virtual bool getBemfParam(int j, double *bemf){return false;} //NOT IMPLEMENTED
    virtual bool setBemfParam(int j, double bemf){return false;} //NOT IMPLEMENTED
    virtual bool setTorquePid(int j, const Pid &pid){return false;} //NOT IMPLEMENTED

    virtual bool getTorque(int j, double *t)
    {
        if (j<_robot_number_of_joints) {
            t[j] = torque[j];
        }
        return true;
    } //NOT TESTED

    virtual bool getTorques(double *t)
    {
        for(unsigned int i = 0; i < _robot_number_of_joints; ++i)
            getTorque(i, t);
        return true;
    } //NOT TESTED

    virtual bool getTorqueRange(int j, double *min, double *max){return false;} //NOT IMPLEMENTED
    virtual bool getTorqueRanges(double *min, double *max){return false;} //NOT IMPLEMENTED
    virtual bool setTorquePids(const Pid *pids){return false;} //NOT IMPLEMENTED
    virtual bool setTorqueErrorLimit(int j, double limit){return false;} //NOT IMPLEMENTED
    virtual bool setTorqueErrorLimits(const double *limits){return false;} //NOT IMPLEMENTED
    virtual bool getTorqueError(int j, double *err){return false;} //NOT IMPLEMENTED
    virtual bool getTorqueErrors(double *errs){return false;} //NOT IMPLEMENTED
    virtual bool getTorquePidOutput(int j, double *out){return false;} //NOT IMPLEMENTED
    virtual bool getTorquePidOutputs(double *outs){return false;} //NOT IMPLEMENTED
    virtual bool getTorquePid(int j, Pid *pid){return false;} //NOT IMPLEMENTED
    virtual bool getTorquePids(Pid *pids){return false;} //NOT IMPLEMENTED
    virtual bool getTorqueErrorLimit(int j, double *limit){return false;} //NOT IMPLEMENTED
    virtual bool getTorqueErrorLimits(double *limits){return false;} //NOT IMPLEMENTED
    virtual bool resetTorquePid(int j){return false;} //NOT IMPLEMENTED
    virtual bool disableTorquePid(int j){return false;} //NOT IMPLEMENTED
    virtual bool enableTorquePid(int j){return false;} //NOT IMPLEMENTED
    virtual bool setTorqueOffset(int j, double v){return false;} //NOT IMPLEMENTED

private:
    unsigned int robot_refresh_period; //ms
    gazebo::physics::Model* _robot;
    gazebo::event::ConnectionPtr updateConnection;
    unsigned int _robot_number_of_joints;

    //Contains the parameters of the device contained in the yarpConfigurationFile .ini file
    yarp::os::Property plugin_parameters;
    
    

    /**
     * The GAZEBO position of each joints, readonly from outside this interface
     */
    yarp::sig::Vector pos;

    /**
     * The zero position is the position of the GAZEBO joint that will be read as the starting one
     * i.e. getEncoder(j)=zero_pos+gazebo.getEncoder(j);
     */
    yarp::sig::Vector zero_pos;

    yarp::sig::Vector vel, speed, acc, amp, torque;
    std::mutex pos_lock;
    yarp::sig::Vector ref_speed, ref_pos, ref_acc, ref_torque;
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
    int _clock;
    int _T_controller;

    /**
     * Private Gazebo stuff
     */
    
    void setMinMaxPos()  //NOT TESTED
    {
        std::cout<<"Joint Limits"<<std::endl;
        for(unsigned int i = 0; i < _robot_number_of_joints; ++i)
        {
            max_pos[i] = this->_robot->GetJoint(joint_names[i])->GetUpperLimit(0).Degree();
            min_pos[i] = this->_robot->GetJoint(joint_names[i])->GetLowerLimit(0).Degree();
            std::cout<<joint_names[i]<<" max_pos: "<<max_pos[i]<<" min_pos: "<<min_pos[i]<<std::endl;
        }
    }

    void setJointNames()  //WORKS
    {
        if( plugin_parameters.check("GAZEBO") ) { 
            std::cout << ".ini file found, using joint names in ini file" << std::endl;
            yarp::os::Bottle joint_names_bottle =plugin_parameters.findGroup("GAZEBO").findGroup("jointNames");
            
            int nr_of_joints = joint_names_bottle.size()-1;
            
            joint_names.resize(nr_of_joints);
            for(int i=0; i < joint_names.size(); i++ ) {
                std::string joint_name(joint_names_bottle.get(i+1).asString().c_str());
                joint_names[i] = _robot->GetName()+"::"+joint_name;
            }
            
        } else {
            std::cout << ".ini file not found, using all the joint names of the robot" << std::endl;
            joint_names.resize(0);
            gazebo::physics::Joint_V joints = _robot->GetJoints();
            int nr_of_joints = _robot->GetJoints().size();
            for(unsigned int i = 0; i < nr_of_joints; ++i)
            {
                gazebo::physics::JointPtr j = joints[i];
                joint_names.push_back(j->GetName());
            }
        }
    }

    void setPIDs() //WORKS
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
        } else if(prop.fromConfigFile(pid_config_abs_path.c_str()))
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

     bool sendPositionsToGazebo(yarp::sig::Vector refs)
    {
        for (int j=0; j<_robot_number_of_joints; j++)
        {
            sendPositionToGazebo(j,refs[j]);
        }
    }

    bool sendPositionToGazebo(int j,double ref)
    {
        gazebo::msgs::JointCmd j_cmd;
        prepareJointMsg(j_cmd,j,ref);
        jointCmdPub->WaitForConnection();
        jointCmdPub->Publish(j_cmd);

    }
    
    void prepareJointMsg(gazebo::msgs::JointCmd& j_cmd, const int joint_index, const double ref)  //WORKS
    {
        j_cmd.set_name(this->_robot->GetJoint(joint_names[joint_index])->GetScopedName());
        j_cmd.mutable_position()->set_target(toRad(ref));
        j_cmd.mutable_position()->set_p_gain(_p[joint_index]);
        j_cmd.mutable_position()->set_i_gain(_i[joint_index]);
        j_cmd.mutable_position()->set_d_gain(_d[joint_index]);
    }
    
    bool sendVelocitiesToGazebo(yarp::sig::Vector& refs) //NOT TESTED
    {
        for (int j=0; j<_robot_number_of_joints; j++)
        {
            sendVelocityToGazebo(j,refs[j]);
        }
    }
    
    bool sendVelocityToGazebo(int j,double ref) //NOT TESTED
    {      
           gazebo::physics::JointPtr joint =  this->_robot->GetJoint(joint_names[j]);
 	   joint->SetMaxForce(0, 200);
// 	   std::cout<<"MaxForce:" <<joint->GetMaxForce(0)<<std::endl;
	   joint->SetVelocity(0,toRad(ref));

//       gazebo::msgs::JointCmd j_cmd;
//       prepareJointVelocityMsg(j_cmd,j,ref);
//       jointCmdPub->WaitForConnection();
//       jointCmdPub->Publish(j_cmd);
    }
    
    void prepareJointVelocityMsg(gazebo::msgs::JointCmd& j_cmd, const int j, const double ref) //NOT TESTED
    {
        j_cmd.set_name(this->_robot->GetJoint(joint_names[j])->GetScopedName());
        j_cmd.mutable_velocity()->set_target(toRad(ref));
        j_cmd.mutable_position()->set_p_gain(0.0);
        j_cmd.mutable_position()->set_i_gain(0.0);
        j_cmd.mutable_position()->set_d_gain(0.0);
        j_cmd.mutable_velocity()->set_p_gain(500);
        j_cmd.mutable_velocity()->set_i_gain(2);
        j_cmd.mutable_velocity()->set_d_gain(0.1);
    }
    
     bool sendTorquesToGazebo(yarp::sig::Vector& refs) //NOT TESTED
    {
        for (int j=0; j<_robot_number_of_joints; j++)
        {
            sendTorqueToGazebo(j,refs[j]);
        }
    }
    
    bool sendTorqueToGazebo(const int j,const double ref) //NOT TESTED
    {
        gazebo::msgs::JointCmd j_cmd;
        prepareJointTorqueMsg(j_cmd,j,ref);
        jointCmdPub->WaitForConnection();
        jointCmdPub->Publish(j_cmd);
    }
    
    void prepareJointTorqueMsg(gazebo::msgs::JointCmd& j_cmd, const int j, const double ref) //NOT TESTED
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

};

#endif //COMAN_H
