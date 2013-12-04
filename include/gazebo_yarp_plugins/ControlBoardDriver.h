/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef __GAZEBO_YARP_CONTROLBOARD_DRIVER_HH__
#define __GAZEBO_YARP_CONTROLBOARD_DRIVER_HH__

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#pragma GCC diagnostic pop

#include "../src/test/jointlogger.hpp"

#define toRad(X) (X*M_PI/180.0)
const double ROBOT_POSITION_TOLERANCE=0.9;

static const std::string pid_config_abs_path = "../config/pid.ini";

namespace yarp {
    namespace dev {
        class GazeboYarpControlBoardDriver;
    }
}

class yarp::dev::GazeboYarpControlBoardDriver : 
    public DeviceDriver,
    public IPositionControl2,
    public IVelocityControl,
    public IAmplifierControl,
    public IEncodersTimed,
    public IControlCalibration2,
    public IControlLimits2,
    public DeviceResponder,
    public IControlMode,
    public ITorqueControl,
    public yarp::os::RateThread
{
public:
    GazeboYarpControlBoardDriver() : RateThread(20)
    {}

    ~GazeboYarpControlBoardDriver(){}

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
    //THREAD (inside comanDeviceDriver.cpp)
    virtual void run();
    virtual bool threadInit();
    virtual void afterStart(bool s);
    virtual void threadRelease();
    
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

    // ENCODERS TIMED
    virtual bool getEncodersTimed(double *encs, double *time);
    virtual bool getEncoderTimed(int j, double *encs, double *time);

    //POSITION CONTROL
    virtual bool stop(int j); //WORKS
    virtual bool stop(); //WORKS
    virtual bool positionMove(int j, double ref); //WORKS
    virtual bool getAxes(int *ax); // WORKS
    virtual bool positionMove(const double *refs); //WORKS
    /// @arg sp [deg/sec]
    virtual bool setRefSpeed(int j, double sp); //WORKS
    virtual bool getRefSpeed(int j, double *ref); //WORKS
    virtual bool getRefSpeeds(double *spds); //WORKS
    
    virtual bool relativeMove(int j, double delta); //NOT TESTED
    virtual bool relativeMove(const double *deltas); //NOT TESTED
    virtual bool checkMotionDone(int j, bool *flag); //NOT TESTED
    virtual bool checkMotionDone(bool *flag); //NOT TESTED
    virtual bool setPositionMode(); //NOT TESTED

    // POS 2
    virtual bool positionMove(const int n_joint, const int *joints, const double *refs);
    virtual bool relativeMove(const int n_joint, const int *joints, const double *deltas);
    virtual bool checkMotionDone(const int n_joint, const int *joints, bool *flags);
    virtual bool setRefSpeeds(const int n_joint, const int *joints, const double *spds);
    virtual bool setRefAccelerations(const int n_joint, const int *joints, const double *accs);
    virtual bool getRefSpeeds(const int n_joint, const int *joints, double *spds);
    virtual bool getRefAccelerations(const int n_joint, const int *joints, double *accs);
    virtual bool stop(const int n_joint, const int *joints);

    /// @arg spds [deg/sec]
    virtual bool setRefSpeeds(const double *spds); //NOT TESTED
    
    virtual bool setRefAcceleration(int j, double acc); //NOT IMPLEMENTED
    virtual bool setRefAccelerations(const double *accs); //NOT IMPLEMENTED
    virtual bool getRefAcceleration(int j, double *acc); //NOT IMPLEMENTED
    virtual bool getRefAccelerations(double *accs); //NOT IMPLEMENTED
    
    //VELOCITY CONTROL
    virtual bool setVelocityMode(); //NOT TESTED
    virtual bool velocityMove(int j, double sp); //NOT TESTED    
    virtual bool velocityMove(const double *sp); //NOT TESTED    
    
    //CONTROL MODE
    virtual bool setPositionMode(int j); //WORKS    
    virtual bool setVelocityMode(int j); //WORKS
    virtual bool getControlMode(int j, int *mode); //WORKS
    
    virtual bool setTorqueMode(int j); //NOT TESTED 
    virtual bool getControlModes(int *modes); //NOT TESTED
    
    virtual bool setImpedancePositionMode(int j);//NOT IMPLEMENTED
    virtual bool setImpedanceVelocityMode(int j); //NOT IMPLEMENTED
    virtual bool setOpenLoopMode(int j); //NOT IMPLEMENTED
    
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
    
    /*
     * Probably useless stuff here
     */
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
    
    // CONTROL LIMITS2 (inside comanOthers.cpp)
    bool getLimits(int axis, double *min, double *max); //WORKS
    bool setLimits(int axis, double min, double max); //WORKS
    bool getVelLimits(int axis, double *min, double *max); //NOT IMPLEMENTED
    bool setVelLimits(int axis, double min, double max); //NOT IMPLEMENTED
    /*
     * End of useless stuff
     */
    
    
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
    yarp::os::Semaphore pos_lock;
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
    
    jointLogger logger;

    yarp::os::Port _joint_torq_port;
    yarp::os::Port _joint_speed_port;

    /**
     * Private Gazebo methods
     */
    void setMinMaxPos();  //NOT TESTED
    void setJointNames();  //WORKS
    void setPIDs(); //WORKS
    bool sendPositionsToGazebo(yarp::sig::Vector refs);
    bool sendPositionToGazebo(int j,double ref);
    void prepareJointMsg(gazebo::msgs::JointCmd& j_cmd, const int joint_index, const double ref);  //WORKS
    bool sendVelocitiesToGazebo(yarp::sig::Vector& refs); //NOT TESTED
    bool sendVelocityToGazebo(int j,double ref); //NOT TESTED
    void prepareJointVelocityMsg(gazebo::msgs::JointCmd& j_cmd, const int j, const double ref); //NOT TESTED
    bool sendTorquesToGazebo(yarp::sig::Vector& refs); //NOT TESTED
    bool sendTorqueToGazebo(const int j,const double ref); //NOT TESTED
    void prepareJointTorqueMsg(gazebo::msgs::JointCmd& j_cmd, const int j, const double ref); //NOT TESTED

};

#endif //COMAN_H
