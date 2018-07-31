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
#include <yarp/dev/IControlMode.h>
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
    public IPositionControl,
    public IVelocityControl,
    public IAmplifierControl,
    public IEncodersTimed,
    public IControlCalibration2,
    public IControlLimits,
    public IInteractionMode,
    public DeviceResponder,
    public IControlMode,
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
    virtual bool getAxisName(int axis, std::string& name);
    virtual bool getJointType(int axis, yarp::dev::JointTypeEnum& type);

    //DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //ENCODERS
    virtual bool getEncoder(int j, double* v) override; //WORKS
    virtual bool getEncoders(double* encs) override; //WORKS
    virtual bool resetEncoder(int j) override; //WORKS
    virtual bool resetEncoders() override; //WORKS
    virtual bool setEncoder(int j, double val) override; //WORKS
    virtual bool setEncoders(const double* vals) override; //WORKS

    virtual bool getEncoderSpeed(int j, double* sp) override; //NOT TESTED
    virtual bool getEncoderSpeeds(double* spds) override; //NOT TESTED

    virtual bool getEncoderAcceleration(int j, double* spds) override;
    virtual bool getEncoderAccelerations(double* accs) override;

    // ENCODERS TIMED
    virtual bool getEncodersTimed(double* encs, double* time) override;
    virtual bool getEncoderTimed(int j, double* encs, double* time) override;

    //POSITION CONTROL
    virtual bool stop(int j) override; //WORKS
    virtual bool stop() override; //WORKS
    virtual bool positionMove(int j, double ref) override; //WORKS
    virtual bool getAxes(int* ax) override; // WORKS
    virtual bool positionMove(const double* refs) override; //WORKS
    /// @arg sp [deg/sec]
    virtual bool setRefSpeed(int j, double sp) override; //WORKS
    virtual bool getRefSpeed(int j, double* ref) override; //WORKS
    virtual bool getRefSpeeds(double* spds) override; //WORKS

    virtual bool relativeMove(int j, double delta) override; //NOT TESTED
    virtual bool relativeMove(const double* deltas) override; //NOT TESTED
    virtual bool checkMotionDone(int j, bool* flag) override; //NOT TESTED
    virtual bool checkMotionDone(bool* flag) override; //NOT TESTED
    
    // Position Control 2
    virtual bool positionMove(const int n_joint, const int* joints, const double* refs) override;
    virtual bool relativeMove(const int n_joint, const int* joints, const double* deltas) override;
    virtual bool checkMotionDone(const int n_joint, const int* joints, bool* flags) override;
    virtual bool setRefSpeeds(const int n_joint, const int* joints, const double* spds) override;
    virtual bool setRefAccelerations(const int n_joint, const int* joints, const double* accs) override;
    virtual bool getRefSpeeds(const int n_joint, const int *joints, double *spds) override;
    virtual bool getRefAccelerations(const int n_joint, const int *joints, double *accs) override;
    virtual bool stop(const int n_joint, const int *joints) override;
    virtual bool getTargetPosition(const int joint, double *ref) override;
    virtual bool getTargetPositions(double *refs) override;
    virtual bool getTargetPositions(const int n_joint, const int *joints, double *refs) override;


    /// @arg spds [deg/sec]
    virtual bool setRefSpeeds(const double *spds) override; //NOT TESTED

    virtual bool setRefAcceleration(int j, double acc) override; //NOT TESTED
    virtual bool setRefAccelerations(const double *accs) override; //NOT TESTED
    virtual bool getRefAcceleration(int j, double *acc) override; //NOT TESTED
    virtual bool getRefAccelerations(double *accs) override; //NOT TESTED

    //VELOCITY CONTROL 2
    virtual bool velocityMove(int j, double sp) override; //NOT TESTED
    virtual bool velocityMove(const double *sp) override; //NOT TESTED
    virtual bool velocityMove(const int n_joint, const int *joints, const double *spds) override;

    virtual bool getRefVelocity(const int joint, double *vel) override;
    virtual bool getRefVelocities(double *vels) override;
    virtual bool getRefVelocities(const int n_joint, const int *joints, double *vels) override;

    //CONTROL MODE
    virtual bool getControlMode(int j, int *mode) override; //WORKS
    virtual bool getControlModes(int *modes) override; //NOT TESTED

    // CONTROL MODE 2
    virtual bool getControlModes(const int n_joint, const int *joints, int *modes) override;
    virtual bool setControlMode(const int j, const int mode) override;
    virtual bool setControlModes(const int n_joint, const int *joints, int *modes) override;
    virtual bool setControlModes(int *modes) override;
    bool changeControlMode(const int j, const int mode); //private function
    bool changeInteractionMode(int j, yarp::dev::InteractionModeEnum mode); //private function

    //TORQUE CONTROL
    virtual bool setRefTorque(int j, double t) override; //NOT TESTED
    virtual bool setRefTorques(const double *t) override; //NOT TESTED
    virtual bool getRefTorque(int j, double *t) override;    //NOT TESTED
    virtual bool getRefTorques(double *t) override;//NOT TESTED
    virtual bool getTorque(int j, double *t) override; //NOT TESTED
    virtual bool getTorques(double *t) override; //NOT TESTED
    virtual bool getTorqueRange(int j, double *min, double *max) override;
    virtual bool getTorqueRanges(double *min, double *max) override;

    //IMPEDANCE CTRL
    virtual bool getImpedance(int j, double *stiffness, double *damping) override; // [Nm/deg] & [Nm*sec/deg]
    virtual bool setImpedance(int j, double stiffness, double damping) override; // [Nm/deg] & [Nm*sec/deg]
    virtual bool setImpedanceOffset(int j, double offset) override;
    virtual bool getImpedanceOffset(int j, double* offset) override;
    virtual bool getCurrentImpedanceLimit(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp) override;

    // PWM interface
    virtual bool getNumberOfMotors(int *ax) override;
    virtual bool setRefDutyCycle(int j, double v) override;
    virtual bool setRefDutyCycles(const double *v) override;
    virtual bool getRefDutyCycle(int j, double *v) override;
    virtual bool getRefDutyCycles(double *v) override;
    virtual bool getDutyCycle(int j, double *v) override;
    virtual bool getDutyCycles(double *v) override;

    // Current interface
    //virtual bool getAxes(int *ax);
    //virtual bool getCurrentRaw(int j, double *t);
    //virtual bool getCurrentsRaw(double *t);
    virtual bool getCurrentRange(int j, double *min, double *max) override;
    virtual bool getCurrentRanges(double *min, double *max) override;
    virtual bool setRefCurrents(const double *t) override;
    virtual bool setRefCurrent(int j, double t) override;
    virtual bool setRefCurrents(const int n_joint, const int *joints, const double *t) override;
    virtual bool getRefCurrents(double *t) override;
    virtual bool getRefCurrent(int j, double *t) override;

    /*
     * IPidControl Interface methods
     */
    virtual bool setPid (const PidControlTypeEnum& pidtype, int j, const Pid &pid) override;
    virtual bool setPids (const PidControlTypeEnum& pidtype, const Pid *pids) override;
    virtual bool setPidReference (const PidControlTypeEnum& pidtype, int j, double ref) override;
    virtual bool setPidReferences (const PidControlTypeEnum& pidtype, const double *refs) override;
    virtual bool setPidErrorLimit (const PidControlTypeEnum& pidtype, int j, double limit) override;
    virtual bool setPidErrorLimits (const PidControlTypeEnum& pidtype, const double *limits) override;
    virtual bool getPidError (const PidControlTypeEnum& pidtype, int j, double *err) override;
    virtual bool getPidErrors (const PidControlTypeEnum& pidtype, double *errs) override;
    virtual bool getPid (const PidControlTypeEnum& pidtype, int j, Pid *pid) override;
    virtual bool getPids (const PidControlTypeEnum& pidtype, Pid *pids) override;
    virtual bool getPidReference (const PidControlTypeEnum& pidtype, int j, double *ref) override;
    virtual bool getPidReferences (const PidControlTypeEnum& pidtype, double *refs) override;
    virtual bool getPidErrorLimit (const PidControlTypeEnum& pidtype, int j, double *limit) override;
    virtual bool getPidErrorLimits (const PidControlTypeEnum& pidtype, double *limits) override;
    virtual bool resetPid (const PidControlTypeEnum& pidtype, int j) override;
    virtual bool disablePid (const PidControlTypeEnum& pidtype, int j) override;
    virtual bool enablePid (const PidControlTypeEnum& pidtype, int j) override;
    virtual bool setPidOffset (const PidControlTypeEnum& pidtype, int j, double v) override;
    virtual bool getPidOutput(const PidControlTypeEnum& pidtype, int j, double *out) override;
    virtual bool getPidOutputs(const PidControlTypeEnum& pidtype, double *outs) override;
    virtual bool isPidEnabled(const PidControlTypeEnum& pidtype, int j, bool* enabled) override;

    /*
     * Probably useless stuff here
     */
    //AMPLIFIER CONTROL (inside comanOthers.cpp)
    virtual bool enableAmp(int j) override;
    virtual bool disableAmp(int j) override;
    virtual bool getCurrent(int j, double *val) override;
    virtual bool getCurrents(double *vals) override;
    virtual bool setMaxCurrent(int j, double v) override;
    virtual bool getMaxCurrent(int j, double *v) override;
    virtual bool getAmpStatus(int *st) override;
    virtual bool getAmpStatus(int k, int *v) override;

    //CONTROL CALIBRATION (inside comanOthers.cpp)
    virtual bool calibrateAxisWithParams(int j, unsigned int iv, double v1, double v2, double v3) override;
    virtual bool calibrationDone(int j) override; // NOT IMPLEMENTED

    /*
     * End of useless stuff
     */

    // RemoteVariables Interface
    virtual bool getRemoteVariable(std::string key, yarp::os::Bottle& val) override;
    virtual bool setRemoteVariable(std::string key, const yarp::os::Bottle& val) override;
    virtual bool getRemoteVariablesList(yarp::os::Bottle* listOfKeys) override;

    // CONTROL LIMITS2 (inside comanOthers.cpp)
    virtual bool getLimits(int axis, double *min, double *max) override;
    virtual bool setLimits(int axis, double min, double max) override;
    virtual bool getVelLimits(int axis, double *min, double *max) override;
    virtual bool setVelLimits(int axis, double min, double max) override;

    // IPOSITION DIRECT
    virtual bool setPosition(int j, double ref) override;
    virtual bool setPositions(const int n_joint, const int *joints, const double *refs) override;
    virtual bool setPositions(const double *refs) override;

    // INTERACTION MODE interface
    virtual bool getInteractionMode(int axis, yarp::dev::InteractionModeEnum* mode) override;
    virtual bool getInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes) override;
    virtual bool getInteractionModes(yarp::dev::InteractionModeEnum* modes) override;
    virtual bool setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode) override;
    virtual bool setInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes) override;
    virtual bool setInteractionModes(yarp::dev::InteractionModeEnum* modes) override;

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
