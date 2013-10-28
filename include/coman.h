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
    public IControlMode
    //,private yarp::os::RateThread
{
public:
    coman()//:RateThread(0)
    {
        //almost everything is done in the open() method
    }

    ~coman()
    {
        delete [] control_mode;
        delete [] motion_done;
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
            for (int j=0; j<_robot_number_of_joints; j++)
            {   //	  temp[j]=0;
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
            pos[j]=joint->GetAngle(0).Degree();  //TODO: if zero_pos=0, it works, if zero_pos=pos[j], pos[j] return 0, if zero_pos=k, pos[j]return 0-k, but since it is an angle, you may get 2*pi
            //std::cout<<"joint"<<j<<" pos"<<pos[j]<<std::endl;
            j++;
        }
        pos_lock.unlock();
        // send positions to the actuators
        _clock++;
        for(int j=0; j<_robot_number_of_joints; ++j)
	{
	    if (control_mode[j]==VOCAB_CM_POSITION)
	    {
	      sendPositionToGazebo(j,ref_pos[j]);
	    }
            if (control_mode[j]==VOCAB_CM_VELOCITY) //TODO check if VOCAB_CM_POSITION or VOCAB_CM_VELOCITY
            {
                if (_clock%100==0)
                {
		  double temp=pos[j];
                    if ( (pos[j]-ref_pos[j]) < -ROBOT_POSITION_TOLERANCE)
                    {
                        temp=pos[j]+ref_speed[j]*robot_refresh_period/10.0;
                        motion_done[j]=false;
                    }
                    else if ( (pos[j]-ref_pos[j]) >ROBOT_POSITION_TOLERANCE)
                    {
                        temp=pos[j]-ref_speed[j]*robot_refresh_period/10.0;
                        motion_done[j]=false;
                    }
                    else
                        motion_done[j]=true;
                std::cout<<"pos: "<<pos[j]<<" ref_pos: "<<ref_pos[j]<<" ref_speed: "<<ref_speed[j]<<" period: "<<robot_refresh_period<<" result: "<<temp<<std::endl;
		sendPositionToGazebo(j,temp);
		}
	    }  
        }
    }

    // thread stuff
    /*virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();
    */
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

    virtual bool setPositionMode() //NOT IMPLEMENTED
    {
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


    virtual bool setRefSpeed(int j, double sp) //WORKS
    {
        if (j<_robot_number_of_joints) {
            ref_speed[j] = sp;
        }
        return true;
    }

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

    virtual bool velocityMove(int j, double sp) //NOT IMPLEMENTED
    {
        if (j<_robot_number_of_joints) {
            vel[j] = sp;
        }
        return true;
    }

    virtual bool velocityMove(const double *sp) //NOT IMPLEMENTED
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
        std::cout<<"control mode = position"<<j<<std::endl;
    }
    virtual bool setVelocityMode(int j) //WORKS
    {
        control_mode[j]=VOCAB_CM_VELOCITY;
        std::cout<<"control mode = speed"<<j<<std::endl;
    }

    virtual bool setVelocityMode() //NOT TESTED
    {
        for(int j=0; j<_robot_number_of_joints; j++)
        {
            control_mode[j]=VOCAB_CM_VELOCITY;
            std::cout<<"control mode = speed for all joints"<<std::endl;
        }
    }

    virtual bool setTorqueMode(int j) //NOT IMPLEMENTED
    {
        return false;
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
    int _clock;

    /**
     * Private Gazebo stuff
     */
    
    void setMinMaxPos()  //NOT TESTED
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
    
    void prepareJointMsg(gazebo::msgs::JointCmd& j_cmd, int joint_index, const double ref)  //WORKS
    {
        j_cmd.set_name(this->_robot->GetJoint(joint_names[joint_index])->GetScopedName());
        j_cmd.mutable_position()->set_target(toRad(ref));
        j_cmd.mutable_position()->set_p_gain(_p[joint_index]);
        j_cmd.mutable_position()->set_i_gain(_i[joint_index]);
        j_cmd.mutable_position()->set_d_gain(_d[joint_index]);
    }
};

#endif //COMAN_H
