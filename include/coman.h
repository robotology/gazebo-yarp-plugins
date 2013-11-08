/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Mingo Enrico, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
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
    public ITorqueControl,
    public yarp::os::RateThread
{
public:
    coman() : RateThread(robot_refresh_period)
    {}

    ~coman(){}

    /**
     * Gazebo stuff
     */
    void gazebo_init();
    void onUpdate(const gazebo::common::UpdateInfo & /*_info*/);

    /**
     * Yarp interfaces start here
     */
    
    
    //DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config);    
    virtual bool close(); //NOT IMPLEMENTED
    //THREAD inside comanDeviceDriver.cpp
    virtual void run();
    virtual bool threadInit();
    virtual void afterStart(bool s);
    virtual void threadRelease();
    
    
    //AMPLIFIER CONTROL (inside comanOthers.cpp)
    virtual bool enableAmp(int j); //NOT IMPLEMENTED
    virtual bool disableAmp(int j); //NOT IMPLEMENTED
    virtual bool getCurrent(int j, double *val); //NOT IMPLEMENTED
    virtual bool getCurrents(double *vals); //NOT IMPLEMENTED
    virtual bool setMaxCurrent(int j, double v); //NOT IMPLEMENTED
    virtual bool getAmpStatus(int *st); //NOT IMPLEMENTED
    virtual bool getAmpStatus(int k, int *v); //NOT IMPLEMENTED
    
    //CONTROL CALIBRATION (inside comanOthers.cpp)
    virtual bool calibrate2(int j, unsigned int iv, double v1, double v2, double v3); //NOT IMPLEMENTED
    virtual bool done(int j); // NOT IMPLEMENTED
    
    // CONTROL LIMITS (inside comanOthers.cpp)
    virtual bool getLimits(int axis, double *min, double *max); //NOT TESTED
    virtual bool setLimits(int axis, double min, double max); //NOT TESTED
    
    //ENCODERS
    virtual bool getEncoder(int j, double *v); //WORKS
    virtual bool getEncoders(double *encs); //WORKS    
    virtual bool resetEncoder(int j); //WORKS
    virtual bool resetEncoders(); //WORKS
    virtual bool setEncoder(int j, double val); //WORKS
    virtual bool setEncoders(const double *vals); //WORKS 
    virtual bool getEncoderSpeed(int j, double *sp); //NOT TESTED
    virtual bool getEncoderSpeeds(double *spds); //NOT TESTED 
    virtual bool getEncoderAcceleration(int j, double *spds); //NOT IMPLEMENTED
    virtual bool getEncoderAccelerations(double *accs); //NOT IMPLEMENTED

    //POSITION CONTROL
    virtual bool stop(int j); //WORKS
    virtual bool stop(); //WORKS
    virtual bool positionMove(int j, double ref); //WORKS
    virtual bool getAxes(int *ax); // WORKS
    virtual bool positionMove(const double *refs); //WORKS
    virtual bool relativeMove(int j, double delta); //NOT TESTED
    virtual bool relativeMove(const double *deltas); //NOT TESTED
    virtual bool checkMotionDone(int j, bool *flag); //NOT TESTED
    virtual bool checkMotionDone(bool *flag); //NOT TESTED
    virtual bool setPositionMode(); //NOT TESTED
    /// @arg sp [deg/sec]
    virtual bool setRefSpeed(int j, double sp); //WORKS
    /// @arg spds [deg/sec]
    virtual bool setRefSpeeds(const double *spds); //NOT TESTED
    virtual bool setRefAcceleration(int j, double acc); //NOT IMPLEMENTED
    virtual bool setRefAccelerations(const double *accs); //NOT IMPLEMENTED
    virtual bool getRefSpeed(int j, double *ref); //WORKS
    virtual bool getRefSpeeds(double *spds); //WORKS
    virtual bool getRefAcceleration(int j, double *acc); //NOT IMPLEMENTED
    virtual bool getRefAccelerations(double *accs); //NOT IMPLEMENTED


    //VELOCITY CONTROL
    virtual bool setVelocityMode(); //NOT TESTED
    virtual bool velocityMove(int j, double sp); //NOT TESTED    
    virtual bool velocityMove(const double *sp); //NOT TESTED    
    
    //CONTROL MODE
    virtual bool setPositionMode(int j); //WORKS    
    virtual bool setVelocityMode(int j); //WORKS
    virtual bool setTorqueMode(int j); //NOT TESTED 
    virtual bool setImpedancePositionMode(int j);//NOT IMPLEMENTED
    virtual bool setImpedanceVelocityMode(int j); //NOT IMPLEMENTED
    virtual bool setOpenLoopMode(int j); //NOT IMPLEMENTED
    virtual bool getControlMode(int j, int *mode); //WORKS
    virtual bool getControlModes(int *modes);
    
    //TORQUE CONTROL
    virtual bool setRefTorque(int j, double t); //NOT TESTED
    virtual bool setRefTorques(const double *t); //NOT TESTED
    virtual bool setTorqueMode(); //NOT TESTED
    virtual bool getRefTorque(int j, double *t);    //NOT TESTED
    virtual bool getRefTorques(double *t);//NOT TESTED
    virtual bool getTorque(int j, double *t); //NOT TESTED
    virtual bool getTorques(double *t); //NOT TESTED
    virtual bool getBemfParam(int j, double *bemf); //NOT IMPLEMENTED
    virtual bool setBemfParam(int j, double bemf); //NOT IMPLEMENTED
    virtual bool setTorquePid(int j, const Pid &pid); //NOT IMPLEMENTED
    virtual bool getTorqueRange(int j, double *min, double *max); //NOT IMPLEMENTED
    virtual bool getTorqueRanges(double *min, double *max); //NOT IMPLEMENTED
    virtual bool setTorquePids(const Pid *pids); //NOT IMPLEMENTED
    virtual bool setTorqueErrorLimit(int j, double limit); //NOT IMPLEMENTED
    virtual bool setTorqueErrorLimits(const double *limits); //NOT IMPLEMENTED
    virtual bool getTorqueError(int j, double *err); //NOT IMPLEMENTED
    virtual bool getTorqueErrors(double *errs); //NOT IMPLEMENTED
    virtual bool getTorquePidOutput(int j, double *out); //NOT IMPLEMENTED
    virtual bool getTorquePidOutputs(double *outs); //NOT IMPLEMENTED
    virtual bool getTorquePid(int j, Pid *pid); //NOT IMPLEMENTED
    virtual bool getTorquePids(Pid *pids); //NOT IMPLEMENTED
    virtual bool getTorqueErrorLimit(int j, double *limit); //NOT IMPLEMENTED
    virtual bool getTorqueErrorLimits(double *limits); //NOT IMPLEMENTED
    virtual bool resetTorquePid(int j); //NOT IMPLEMENTED
    virtual bool disableTorquePid(int j); //NOT IMPLEMENTED
    virtual bool enableTorquePid(int j); //NOT IMPLEMENTED
    virtual bool setTorqueOffset(int j, double v); //NOT IMPLEMENTED

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

    yarp::os::Port _joint_torq_port;
    yarp::os::Port _joint_speed_port;

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
        /* SetVelocity method */
        gazebo::physics::JointPtr joint =  this->_robot->GetJoint(joint_names[j]);
        joint->SetMaxForce(0, joint->GetEffortLimit(0)*1.1); //<-- MAGIC NUMBER!!!!
//      std::cout<<"MaxForce:" <<joint->GetMaxForce(0)<<std::endl;
        joint->SetVelocity(0,toRad(ref));

        /* JointController method. If you pick this control method for control
           of joint velocities, you should also take care of the switching logic
           in setVelocityMode, setTorqueMode and setPositionMode:
           that is, the SetMarxForce(0,0) and SetVelocity(0,0) are no longer
           needed, but the JointController::AddJoint() method needs to be called
           when you switch to velocity mode, to make sure the PIDs get reset */
//       gazebo::msgs::JointCmd j_cmd;
//       prepareJointVelocityMsg(j_cmd,j,ref);
//       jointCmdPub->WaitForConnection();
//       jointCmdPub->Publish(j_cmd);
    }
    
    void prepareJointVelocityMsg(gazebo::msgs::JointCmd& j_cmd, const int j, const double ref) //NOT TESTED
    {
        j_cmd.set_name(this->_robot->GetJoint(joint_names[j])->GetScopedName());
        j_cmd.mutable_position()->set_p_gain(0.0);
        j_cmd.mutable_position()->set_i_gain(0.0);
        j_cmd.mutable_position()->set_d_gain(0.0);
        j_cmd.mutable_velocity()->set_p_gain(5000);
        j_cmd.mutable_velocity()->set_i_gain(0.0);
        j_cmd.mutable_velocity()->set_d_gain(10);
        j_cmd.mutable_velocity()->set_target(toRad(ref));
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
