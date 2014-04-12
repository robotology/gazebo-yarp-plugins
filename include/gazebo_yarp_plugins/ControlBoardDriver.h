/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef GAZEBOYARP_CONTROLBOARDDRIVER_HH
#define GAZEBOYARP_CONTROLBOARDDRIVER_HH

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IOpenLoopControl.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>


const double ROBOT_POSITION_TOLERANCE = 0.9;

namespace yarp {
    namespace dev {
        class GazeboYarpControlBoardDriver;
    }

}


class yarp::dev::GazeboYarpControlBoardDriver:
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
    public IPositionDirect,
    public IImpedanceControl,
    public IOpenLoopControl,
    public IPidControl
{
public:

    GazeboYarpControlBoardDriver();

    virtual ~GazeboYarpControlBoardDriver();

    /**
     * Gazebo stuff
     */
    bool gazebo_init();
    void onUpdate(const gazebo::common::UpdateInfo & /*_info*/);

    /**
     * Yarp interfaces start here
     */

    //DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

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

    //IMPEDANCE CTRL
    virtual bool getImpedance(int j, double *stiffness, double *damping); // [Nm/deg] & [Nm*sec/deg]
    virtual bool setImpedance(int j, double stiffness, double damping); // [Nm/deg] & [Nm*sec/deg]
    virtual bool setImpedanceOffset(int j, double offset);
    virtual bool getImpedanceOffset(int j, double* offset);
    virtual bool getCurrentImpedanceLimit(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp);

    //IOpenLoopControl interface methods
    /**
     * Command direct output value to joint j. Currently this is a torque
     * \param j joint number
     * \param v value to be set
     * \return true if the operation succeeded. False otherwise
     */
    virtual bool setOutput(int j, double v);
    virtual bool setOutputs(const double *v);
    virtual bool getOutput(int j, double *v);
    virtual bool getOutputs(double *v);
    virtual bool setOpenLoopMode();

    /*
     * IPidControl Interface methods
     */
    virtual bool setPid (int j, const Pid &pid);
    virtual bool setPids (const Pid *pids);
    virtual bool setReference (int j, double ref);
    virtual bool setReferences (const double *refs);
    virtual bool setErrorLimit (int j, double limit);
    virtual bool setErrorLimits (const double *limits);
    virtual bool getError (int j, double *err);
    virtual bool getErrors (double *errs);
    virtual bool getPid (int j, Pid *pid);
    virtual bool getPids (Pid *pids);
    virtual bool getReference (int j, double *ref);
    virtual bool getReferences (double *refs);
    virtual bool getErrorLimit (int j, double *limit);
    virtual bool getErrorLimits (double *limits);
    virtual bool resetPid (int j);
    virtual bool disablePid (int j);
    virtual bool enablePid (int j);
    virtual bool setOffset (int j, double v);

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

    // IPOSITION DIRECT
    bool setPositionDirectMode();
    bool setPosition(int j, double ref);
    bool setPositions(const int n_joint, const int *joints, double *refs);
    bool setPositions(const double *refs);

private:

    /* PID structures */
    struct PID {
        double p;
        double i;
        double d;
        double maxInt;
        double maxOut;
    };

    enum PIDFeedbackTerm {
        PIDFeedbackTermProportionalTerm = 1,
        PIDFeedbackTermIntegrativeTerm = 1 << 1,
        PIDFeedbackTermDerivativeTerm = 1 << 2,
        PIDFeedbackTermAllTerms = PIDFeedbackTermProportionalTerm | PIDFeedbackTermIntegrativeTerm | PIDFeedbackTermDerivativeTerm
    };

    unsigned int robotRefreshPeriod; //ms
    gazebo::physics::Model* _robot;
    gazebo::event::ConnectionPtr updateConnection;
    unsigned int numberOfJoints;

    //Contains the parameters of the device contained in the yarpConfigurationFile .ini file
    yarp::os::Property pluginParameters;

    /**
     * The GAZEBO position of each joints, readonly from outside this interface
     */
    yarp::sig::Vector pos;
    /**
     * The GAZEBO desired position of each joints, (output of trajectory interp)
     */
    yarp::sig::Vector desiredPosition;

    /**
     * The zero position is the position of the GAZEBO joint that will be read as the starting one
     * i.e. getEncoder(j)=zeroPosition+gazebo.getEncoder(j);
     */
    yarp::sig::Vector zeroPosition;

    yarp::sig::Vector vel, speed, acc, amp, torque;
    yarp::os::Semaphore pos_lock;
    yarp::sig::Vector referenceSpeed, referencePosition, referenceAcceleraton, referenceTorque;
    yarp::sig::Vector max_pos, min_pos;
//    yarp::sig::ImageOf<yarp::sig::PixelRgb> back, fore;

    std::vector<std::string> joint_names;
    gazebo::transport::NodePtr gazeboNode;
    gazebo::transport::PublisherPtr jointCmdPub;
    std::vector<GazeboYarpControlBoardDriver::PID> _positionPIDs;
    std::vector<GazeboYarpControlBoardDriver::PID> _velocityPIDs;
    std::vector<GazeboYarpControlBoardDriver::PID> _impedancePosPDs;

    yarp::sig::Vector torqueOffsett;
    yarp::sig::Vector minStiffness;
    yarp::sig::Vector minDamping;
    yarp::sig::Vector maxStiffness;
    yarp::sig::Vector maxDamping;

    bool *motion_done;
    int  *controlMode;
    bool command_changed;
    bool started;
    int _clock;
    int _T_controller;

    //jointLogger logger;

    /**
     * Private Gazebo methods
     */
    void setMinMaxPos();  //NOT TESTED
    bool setJointNames();  //WORKS
    void setPIDsForGroup(std::string, std::vector<GazeboYarpControlBoardDriver::PID>&, enum PIDFeedbackTerm pidTerms);
    void setMinMaxImpedance();
    void setPIDs(); //WORKS
    bool sendPositionsToGazebo(yarp::sig::Vector& refs);
    bool sendPositionToGazebo(int j,double ref);
    void prepareJointMsg(gazebo::msgs::JointCmd& j_cmd, const int joint_index, const double ref);  //WORKS
    bool sendVelocitiesToGazebo(yarp::sig::Vector& refs); //NOT TESTED
    bool sendVelocityToGazebo(int j,double ref); //NOT TESTED
    void prepareJointVelocityMsg(gazebo::msgs::JointCmd& j_cmd, const int j, const double ref); //NOT TESTED
    bool sendTorquesToGazebo(yarp::sig::Vector& refs); //NOT TESTED
    bool sendTorqueToGazebo(const int j,const double ref); //NOT TESTED
    void prepareJointTorqueMsg(gazebo::msgs::JointCmd& j_cmd, const int j, const double ref); //NOT TESTED
    void sendImpPositionToGazebo ( const int j, const double des );
    void sendImpPositionsToGazebo ( yarp::sig::Vector& dess );
    void computeTrajectory(const int j);
    void prepareResetJointMsg(int j);

};

#endif //GAZEBOYARP_CONTROLBOARDDRIVER_HH
