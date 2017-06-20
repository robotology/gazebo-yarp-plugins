/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_FAKECONTROLBOARDDRIVER_HH
#define GAZEBOYARP_FAKECONTROLBOARDDRIVER_HH

#include <yarp/os/Property.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IPWMControl.h>
#include <yarp/dev/ICurrentControl.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IControlMode2.h>
#include <yarp/dev/IInteractionMode.h>
#include <yarp/dev/IRemoteVariables.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

extern const double RobotPositionTolerance;

namespace yarp {
    namespace dev {
        class GazeboYarpFakeControlBoardDriver;
    }
}

namespace gazebo {
    namespace common {
        class UpdateInfo;
    }

    namespace physics {
        class Model;
        class Joint;
        typedef boost::shared_ptr<Joint> JointPtr;
    }

    namespace event {
        class Connection;
        typedef boost::shared_ptr<Connection> ConnectionPtr;
    }

    namespace transport {
        class Node;
        class Publisher;

        typedef boost::shared_ptr<Node> NodePtr;
        typedef boost::shared_ptr<Publisher> PublisherPtr;
    }

    namespace msgs {
        class JointCmd;
    }
}

class yarp::dev::GazeboYarpFakeControlBoardDriver:
    public DeviceDriver,
    public IPositionControl2,
    public IVelocityControl2,
    public IAmplifierControl,
    public IEncodersTimed,
    public IControlCalibration2,
    public IControlLimits2,
    public IInteractionMode,
    public DeviceResponder,
    public IControlMode2,
    public ITorqueControl,
    public IPositionDirect,
    public IImpedanceControl,
    public IPWMControl,
    public ICurrentControl,
    public IPidControl,
    public IRemoteVariables,
    public IAxisInfo
{
public:
    GazeboYarpFakeControlBoardDriver();
    virtual ~GazeboYarpFakeControlBoardDriver();

    /**
     * Callback for the WorldUpdateBegin Gazebo event.
     */
    void onUpdate(const gazebo::common::UpdateInfo&);

    /**
     * Helper methods
     */
    bool getZero(int j,  double* val);
    bool getZero(double* vals);
    bool getZero(const int n_joint, const int* joints,  double* refs);
    bool getTrue(int j, bool* flag);
    bool getTrue(bool* flags);
    bool getTrueIfArgumentIsZero(const int n_joint);

    /**
     * Yarp interfaces start here
     */

    // AXIS IAxisInfo
    virtual bool getAxisName(int axis, yarp::os::ConstString& name);
    virtual bool getJointType(int axis, yarp::dev::JointTypeEnum& type);

    //DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //ENCODERS
    virtual bool getEncoder(int j, double* v); //WORKS
    virtual bool getEncoders(double* encs); //WORKS
    virtual bool resetEncoder(int j); //WORKS
    virtual bool resetEncoders(); //WORKS
    virtual bool setEncoder(int j, double val); //WORKS
    virtual bool setEncoders(const double* vals); //WORKS

    virtual bool getEncoderSpeed(int j, double* sp); //NOT TESTED
    virtual bool getEncoderSpeeds(double* spds); //NOT TESTED

    virtual bool getEncoderAcceleration(int j, double* spds);
    virtual bool getEncoderAccelerations(double* accs);

    // ENCODERS TIMED
    virtual bool getEncodersTimed(double* encs, double* time);
    virtual bool getEncoderTimed(int j, double* encs, double* time);

    //POSITION CONTROL
    virtual bool stop(int j); //WORKS
    virtual bool stop(); //WORKS
    virtual bool positionMove(int j, double ref); //WORKS
    virtual bool getAxes(int* ax); // WORKS
    virtual bool positionMove(const double* refs); //WORKS
    /// @arg sp [deg/sec]
    virtual bool setRefSpeed(int j, double sp); //WORKS
    virtual bool getRefSpeed(int j, double* ref); //WORKS
    virtual bool getRefSpeeds(double* spds); //WORKS

    virtual bool relativeMove(int j, double delta); //NOT TESTED
    virtual bool relativeMove(const double* deltas); //NOT TESTED
    virtual bool checkMotionDone(int j, bool* flag); //NOT TESTED
    virtual bool checkMotionDone(bool* flag); //NOT TESTED
    virtual bool setPositionMode(); //NOT TESTED

    // Position Control 2
    virtual bool positionMove(const int n_joint, const int* joints, const double* refs);
    virtual bool relativeMove(const int n_joint, const int* joints, const double* deltas);
    virtual bool checkMotionDone(const int n_joint, const int* joints, bool* flags);
    virtual bool setRefSpeeds(const int n_joint, const int* joints, const double* spds);
    virtual bool setRefAccelerations(const int n_joint, const int* joints, const double* accs);
    virtual bool getRefSpeeds(const int n_joint, const int *joints, double *spds);
    virtual bool getRefAccelerations(const int n_joint, const int *joints, double *accs);
    virtual bool stop(const int n_joint, const int *joints);
    virtual bool getTargetPosition(const int joint, double *ref);
    virtual bool getTargetPositions(double *refs);
    virtual bool getTargetPositions(const int n_joint, const int *joints, double *refs);


    /// @arg spds [deg/sec]
    virtual bool setRefSpeeds(const double *spds); //NOT TESTED

    virtual bool setRefAcceleration(int j, double acc); //NOT TESTED
    virtual bool setRefAccelerations(const double *accs); //NOT TESTED
    virtual bool getRefAcceleration(int j, double *acc); //NOT TESTED
    virtual bool getRefAccelerations(double *accs); //NOT TESTED

    //VELOCITY CONTROL 2
    virtual bool setVelocityMode(); //NOT TESTED
    virtual bool velocityMove(int j, double sp); //NOT TESTED
    virtual bool velocityMove(const double *sp); //NOT TESTED
    virtual bool velocityMove(const int n_joint, const int *joints, const double *spds);

    virtual bool getRefVelocity(const int joint, double *vel);
    virtual bool getRefVelocities(double *vels);
    virtual bool getRefVelocities(const int n_joint, const int *joints, double *vels);

    //CONTROL MODE
    virtual bool setPositionMode(int j); //WORKS
    virtual bool setVelocityMode(int j); //WORKS
    virtual bool getControlMode(int j, int *mode); //WORKS

    virtual bool setTorqueMode(int j); //NOT TESTED
    virtual bool getControlModes(int *modes); //NOT TESTED

    virtual bool setImpedancePositionMode(int j);
    virtual bool setImpedanceVelocityMode(int j);

    // CONTROL MODE 2
    virtual bool getControlModes(const int n_joint, const int *joints, int *modes);
    virtual bool setControlMode(const int j, const int mode);
    virtual bool setControlModes(const int n_joint, const int *joints, int *modes);
    virtual bool setControlModes(int *modes);
    bool changeControlMode(const int j, const int mode); //private function
    bool changeInteractionMode(int j, yarp::dev::InteractionModeEnum mode); //private function

    //TORQUE CONTROL
    virtual bool setRefTorque(int j, double t); //NOT TESTED
    virtual bool setRefTorques(const double *t); //NOT TESTED
    virtual bool setTorqueMode(); //NOT TESTED
    virtual bool getRefTorque(int j, double *t);    //NOT TESTED
    virtual bool getRefTorques(double *t);//NOT TESTED
    virtual bool getTorque(int j, double *t); //NOT TESTED
    virtual bool getTorques(double *t); //NOT TESTED

    virtual bool getBemfParam(int j, double *bemf);
    virtual bool setBemfParam(int j, double bemf);
    virtual bool getTorqueRange(int j, double *min, double *max);
    virtual bool getTorqueRanges(double *min, double *max);
    virtual bool setTorqueOffset(int j, double v);

    //IMPEDANCE CTRL
    virtual bool getImpedance(int j, double *stiffness, double *damping); // [Nm/deg] & [Nm*sec/deg]
    virtual bool setImpedance(int j, double stiffness, double damping); // [Nm/deg] & [Nm*sec/deg]
    virtual bool setImpedanceOffset(int j, double offset);
    virtual bool getImpedanceOffset(int j, double* offset);
    virtual bool getCurrentImpedanceLimit(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp);

    // PWM interface
    virtual bool getNumberOfMotors(int *ax);
    virtual bool setRefDutyCycle(int j, double v);
    virtual bool setRefDutyCycles(const double *v);
    virtual bool getRefDutyCycle(int j, double *v);
    virtual bool getRefDutyCycles(double *v);
    virtual bool getDutyCycle(int j, double *v);
    virtual bool getDutyCycles(double *v);

    // Current interface
    //virtual bool getAxes(int *ax);
    //virtual bool getCurrentRaw(int j, double *t);
    //virtual bool getCurrentsRaw(double *t);
    virtual bool getCurrentRange(int j, double *min, double *max);
    virtual bool getCurrentRanges(double *min, double *max);
    virtual bool setRefCurrents(const double *t);
    virtual bool setRefCurrent(int j, double t);
    virtual bool setRefCurrents(const int n_joint, const int *joints, const double *t);
    virtual bool getRefCurrents(double *t);
    virtual bool getRefCurrent(int j, double *t);

    /*
     * IPidControl Interface methods
     */
    virtual bool setPid (const PidControlTypeEnum& pidtype, int j, const Pid &pid);
    virtual bool setPids (const PidControlTypeEnum& pidtype, const Pid *pids);
    virtual bool setPidReference (const PidControlTypeEnum& pidtype, int j, double ref);
    virtual bool setPidReferences (const PidControlTypeEnum& pidtype, const double *refs);
    virtual bool setPidErrorLimit (const PidControlTypeEnum& pidtype, int j, double limit);
    virtual bool setPidErrorLimits (const PidControlTypeEnum& pidtype, const double *limits);
    virtual bool getPidError (const PidControlTypeEnum& pidtype, int j, double *err);
    virtual bool getPidErrors (const PidControlTypeEnum& pidtype, double *errs);
    virtual bool getPid (const PidControlTypeEnum& pidtype, int j, Pid *pid);
    virtual bool getPids (const PidControlTypeEnum& pidtype, Pid *pids);
    virtual bool getPidReference (const PidControlTypeEnum& pidtype, int j, double *ref);
    virtual bool getPidReferences (const PidControlTypeEnum& pidtype, double *refs);
    virtual bool getPidErrorLimit (const PidControlTypeEnum& pidtype, int j, double *limit);
    virtual bool getPidErrorLimits (const PidControlTypeEnum& pidtype, double *limits);
    virtual bool resetPid (const PidControlTypeEnum& pidtype, int j);
    virtual bool disablePid (const PidControlTypeEnum& pidtype, int j);
    virtual bool enablePid (const PidControlTypeEnum& pidtype, int j);
    virtual bool setPidOffset (const PidControlTypeEnum& pidtype, int j, double v);
    virtual bool getPidOutput(const PidControlTypeEnum& pidtype, int j, double *out);
    virtual bool getPidOutputs(const PidControlTypeEnum& pidtype, double *outs);
    virtual bool isPidEnabled(const PidControlTypeEnum& pidtype, int j, bool* enabled);

    /*
     * Probably useless stuff here
     */
    //AMPLIFIER CONTROL (inside comanOthers.cpp)
    virtual bool enableAmp(int j);
    virtual bool disableAmp(int j);
    virtual bool getCurrent(int j, double *val);
    virtual bool getCurrents(double *vals);
    virtual bool setMaxCurrent(int j, double v);
    virtual bool getMaxCurrent(int j, double *v);
    virtual bool getAmpStatus(int *st);
    virtual bool getAmpStatus(int k, int *v);

    //CONTROL CALIBRATION (inside comanOthers.cpp)
    virtual bool calibrate2(int j, unsigned int iv, double v1, double v2, double v3);
    virtual bool done(int j); // NOT IMPLEMENTED

    /*
     * End of useless stuff
     */

    // RemoteVariables Interface
    virtual bool getRemoteVariable(yarp::os::ConstString key, yarp::os::Bottle& val);
    virtual bool setRemoteVariable(yarp::os::ConstString key, const yarp::os::Bottle& val);
    virtual bool getRemoteVariablesList(yarp::os::Bottle* listOfKeys);

    // CONTROL LIMITS2 (inside comanOthers.cpp)
    virtual bool getLimits(int axis, double *min, double *max);
    virtual bool setLimits(int axis, double min, double max);
    virtual bool getVelLimits(int axis, double *min, double *max);
    virtual bool setVelLimits(int axis, double min, double max);

    // IPOSITION DIRECT
    virtual bool setPositionDirectMode();
    virtual bool setPosition(int j, double ref);
    virtual bool setPositions(const int n_joint, const int *joints, double *refs);
    virtual bool setPositions(const double *refs);

    // INTERACTION MODE interface
    virtual bool getInteractionMode(int axis, yarp::dev::InteractionModeEnum* mode);
    virtual bool getInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    virtual bool getInteractionModes(yarp::dev::InteractionModeEnum* modes);
    virtual bool setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode);
    virtual bool setInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    virtual bool setInteractionModes(yarp::dev::InteractionModeEnum* modes);

private:
    yarp::os::Property m_pluginParameters; /**< Contains the parameters of the device contained in the yarpConfigurationFile .ini file */
    unsigned int m_numberOfJoints; /**< number of joints controlled by the control board */
    yarp::sig::Vector m_positions; /**< joint positions [Degrees] */
    yarp::sig::Vector m_velocities; /**< joint velocities [Degrees/Seconds] */
    yarp::sig::Vector m_torques; /**< joint torques [Netwon Meters] */
    yarp::sig::Vector m_zeroPosition;
    std::vector<std::string> m_jointNames;
    std::vector<yarp::dev::JointTypeEnum> m_jointTypes;

    std::vector<int> m_controlMode;
    std::vector<int>  m_interactionMode;

    // Attributes related to the updating on the timestamp

    /**
     * Connection to the WorldUpdateBegin Gazebo event
     */
    gazebo::event::ConnectionPtr m_updateConnection;
    yarp::os::Mutex m_lastTimestampMutex; /**< Mutex protecting the m_lastTimestamp variable. */
    yarp::os::Stamp m_lastTimestamp; /**< timestamp, updated with simulation time at each onUpdate call */
};

#endif //GAZEBOYARP_CONTROLBOARDDRIVER_HH
