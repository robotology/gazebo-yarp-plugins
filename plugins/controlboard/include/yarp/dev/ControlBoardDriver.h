/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_CONTROLBOARDDRIVER_HH
#define GAZEBOYARP_CONTROLBOARDDRIVER_HH

#include <yarp/os/Property.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IPidControl.h>
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

#include <boost/shared_ptr.hpp>
#include <ControlBoardDriverTrajectory.h>
#include <ControlBoardDriverCoupling.h>

#include <gazebo/common/PID.hh>
#include <gazebo/common/Time.hh>

#include <string>
#include <functional>
#include <unordered_map>
#include <vector>


extern const double RobotPositionTolerance;

namespace yarp {
    namespace dev {
        class GazeboYarpControlBoardDriver;

        struct PidControlTypeEnumHashFunction {
            size_t operator() (const PidControlTypeEnum& key) const {
                std::size_t hash = std::hash<int>()(static_cast<int>(key));
                return hash;
            }
        };


//
//        bool PidControlTypeEnumCompareFunction(const PidControlTypeEnum& key1, const PidControlTypeEnum& key2)  {
//            return key1 == key2;
//        }

    }
}

namespace gazebo {
    namespace common {
        class UpdateInfo;
        class PID;
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
}

class yarp::dev::GazeboYarpControlBoardDriver:
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

    GazeboYarpControlBoardDriver();
    virtual ~GazeboYarpControlBoardDriver();

    /**
     * Gazebo stuff
     */
    bool gazebo_init();

    /**
     * Callback for the WorldUpdateBegin Gazebo event.
     */
    void onUpdate(const gazebo::common::UpdateInfo&);

    /**
     * Callback for the WorldReset Gazebo event.
     */
    void onReset();

    /**
     * Helper function for resetting the position and the trajectory generator.
     */
    void resetPositionsAndTrajectoryGenerators();


    /**
     * Yarp interfaces start here
     */

    // AXIS IAxisInfo
    virtual bool getAxisName(int axis, std::string& name) override;
    virtual bool getJointType(int axis, yarp::dev::JointTypeEnum& type) override;

    //DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config) override;
    virtual bool close() override;

    //ENCODERS
    virtual bool getEncoder(int j, double* v) override; //WORKS
    virtual bool getEncoders(double* encs) override; //WORKS
    virtual bool resetEncoder(int j) override; //WORKS
    virtual bool resetEncoders() override; //WORKS
    virtual bool setEncoder(int j, double val) override; //WORKS
    virtual bool setEncoders(const double* vals) override; //WORKS

    virtual bool getEncoderSpeed(int j, double* sp) override; //NOT TESTED
    virtual bool getEncoderSpeeds(double* spds) override; //NOT TESTED

    virtual bool getEncoderAcceleration(int j, double* spds) override; //NOT IMPLEMENTED
    virtual bool getEncoderAccelerations(double* accs) override; //NOT IMPLEMENTED

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
    virtual bool setRefTorque(int j, double t) override;
    virtual bool setRefTorques(const double *t) override;
    virtual bool getRefTorque(int j, double *t) override;
    virtual bool getRefTorques(double *t) override;
    virtual bool setRefTorques(const int n_joint, const int *joints, const double *t) override;
    virtual bool getTorque(int j, double *t) override;
    virtual bool getTorques(double *t) override;

    virtual bool getTorqueRange(int j, double *min, double *max) override;
    virtual bool getTorqueRanges(double *min, double *max) override;
    virtual bool getMotorTorqueParams(int j,  yarp::dev::MotorTorqueParameters *params) override;
    virtual bool setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params) override;

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
    //virtual bool getAxes(int *ax) override;
    virtual bool getCurrent(int j, double *t) override;
    virtual bool getCurrents(double *t) override;
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
    virtual bool setPid(const PidControlTypeEnum& pidtype, int j, const Pid &pid) override;
    virtual bool setPids(const PidControlTypeEnum& pidtype, const Pid *pids) override;
    virtual bool setPidReference(const PidControlTypeEnum& pidtype, int j, double ref) override;
    virtual bool setPidReferences(const PidControlTypeEnum& pidtype, const double *refs) override;
    virtual bool setPidErrorLimit(const PidControlTypeEnum& pidtype, int j, double limit) override;
    virtual bool setPidErrorLimits(const PidControlTypeEnum& pidtype, const double *limits) override;
    virtual bool getPidError(const PidControlTypeEnum& pidtype, int j, double *err) override;
    virtual bool getPidErrors(const PidControlTypeEnum& pidtype, double *errs) override;
    virtual bool getPidOutput(const PidControlTypeEnum& pidtype, int j, double *out) override;
    virtual bool getPidOutputs(const PidControlTypeEnum& pidtype, double *outs) override;
    virtual bool getPid(const PidControlTypeEnum& pidtype, int j, Pid *pid) override;
    virtual bool getPids(const PidControlTypeEnum& pidtype, Pid *pids) override;
    virtual bool getPidReference(const PidControlTypeEnum& pidtype, int j, double *ref) override;
    virtual bool getPidReferences(const PidControlTypeEnum& pidtype, double *refs) override;
    virtual bool getPidErrorLimit(const PidControlTypeEnum& pidtype, int j, double *limit) override;
    virtual bool getPidErrorLimits(const PidControlTypeEnum& pidtype, double *limits) override;
    virtual bool resetPid(const PidControlTypeEnum& pidtype, int j) override;
    virtual bool disablePid(const PidControlTypeEnum& pidtype, int j) override;
    virtual bool enablePid(const PidControlTypeEnum& pidtype, int j) override;
    virtual bool setPidOffset(const PidControlTypeEnum& pidtype, int j, double v) override;
    virtual bool isPidEnabled(const PidControlTypeEnum& pidtype, int j, bool* enabled) override;

    /*
     * Probably useless stuff here
     */
    
    //AMPLIFIER CONTROL (inside comanOthers.cpp)
    virtual bool enableAmp(int j) override; //NOT IMPLEMENTED
    virtual bool disableAmp(int j) override; //NOT IMPLEMENTED
    virtual bool setMaxCurrent(int j, double v) override; //NOT IMPLEMENTED
    virtual bool getMaxCurrent(int j, double *v) override;  //NOT IMPLEMENTED
    virtual bool getAmpStatus(int *st) override; //NOT IMPLEMENTED
    virtual bool getAmpStatus(int k, int *v) override; //NOT IMPLEMENTED

    //CONTROL CALIBRATION (inside comanOthers.cpp)
    virtual bool calibrateAxisWithParams(int j, unsigned int iv, double v1, double v2, double v3) override; //NOT IMPLEMENTED
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
    virtual bool getRefPosition (const int joint, double *ref) override;
    virtual bool getRefPositions (double *refs) override;
    virtual bool getRefPositions (const int n_joint, const int *joints, double *refs) override;

    // INTERACTION MODE interface
    virtual bool getInteractionMode(int axis, yarp::dev::InteractionModeEnum* mode) override;
    virtual bool getInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes) override;
    virtual bool getInteractionModes(yarp::dev::InteractionModeEnum* modes) override;
    virtual bool setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode) override;
    virtual bool setInteractionModes(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes) override;
    virtual bool setInteractionModes(yarp::dev::InteractionModeEnum* modes) override;

private:

    enum PIDFeedbackTerm {
        PIDFeedbackTermProportionalTerm = 1,
        PIDFeedbackTermIntegrativeTerm = 1 << 1,
        PIDFeedbackTermDerivativeTerm = 1 << 2,
        PIDFeedbackTermAllTerms = PIDFeedbackTermProportionalTerm | PIDFeedbackTermIntegrativeTerm | PIDFeedbackTermDerivativeTerm
    };

    enum JointType
    {
        JointType_Unknown = 0,
        JointType_Revolute,
        JointType_Prismatic
    };

    struct Range {
        Range() : min(0), max(0){}
        double min;
        double max;
    };

    std::string m_deviceName;
    gazebo::physics::Model* m_robot;

    /**
     * Connection to the WorldUpdateBegin Gazebo event
     */
    gazebo::event::ConnectionPtr m_updateConnection;

    /**
     * Connection to the WorldReset Gazebo event
     */
    gazebo::event::ConnectionPtr m_resetConnection;




    yarp::os::Property m_pluginParameters; /**< Contains the parameters of the device contained in the yarpConfigurationFile .ini file */

    size_t m_numberOfJoints; /**< number of joints controlled by the control board */
    std::vector<Range> m_jointPosLimits;
    std::vector<Range> m_jointVelLimits;

    /**
     * The zero position is the position of the GAZEBO joint that will be read as the starting one
     * i.e. getEncoder(j)=m_zeroPosition+gazebo.getEncoder(j);
     */
    yarp::sig::Vector m_zeroPosition;

    yarp::sig::Vector m_positions;          /**< joint positions [Degrees] */
    yarp::sig::Vector m_motPositions;      /**< motor positions [Degrees] */
    yarp::sig::Vector m_velocities;         /**< joint velocities [Degrees/Seconds] */
    yarp::sig::Vector m_torques;            /**< joint torques [Netwon Meters] */
    yarp::sig::Vector m_maxTorques;         /**< joint torques [Netwon Meters] */

    yarp::os::Stamp m_lastTimestamp;        /**< timestamp, updated with simulation time at each onUpdate call */

    yarp::sig::VectorOf<JointType> m_jointTypes;
    yarp::sig::Vector m_amp;
    
    //Desired Control variables
    yarp::sig::Vector m_jntReferencePositions; /**< desired reference positions.
                                                 Depending on the position mode,
                                                 they can be set directly or indirectly
                                                 through the trajectory generator.
                                                 [Degrees] */

    yarp::sig::Vector m_motReferencePositions;   //after calling decouple decoupleRefPos this is the reference which is sent to the PID
    yarp::sig::Vector m_motReferenceVelocities;  //after calling decouple decoupleRefVel this is the reference which is sent to the PID
    yarp::sig::Vector m_motReferenceTorques;     //after calling decouple decoupleRefTrq this is the reference which is sent to the PID

    yarp::sig::Vector m_oldReferencePositions; // used to store last reference and check if a new ref has been commanded
    yarp::sig::Vector m_positionThreshold;  // Threshold under which trajectory generator stops computing new values

    yarp::sig::Vector m_jntReferenceTorques; /**< desired reference torques for torque control mode [NetwonMeters] */
    yarp::sig::Vector m_jntReferenceVelocities; /**< desired reference velocities for velocity control mode [Degrees/Seconds] */

    //trajectory generator
    std::vector<TrajectoryGenerator*> m_trajectory_generator;
    std::vector<BaseCouplingHandler*>  m_coupling_handler;
    std::vector<RampFilter*> m_speed_ramp_handler;
    std::vector<Watchdog*> m_velocity_watchdog;

    yarp::sig::Vector m_trajectoryGenerationReferencePosition; /**< reference position for trajectory generation in position mode [Degrees] */
    yarp::sig::Vector m_trajectoryGenerationReferenceSpeed; /**< reference speed for trajectory generation in position mode [Degrees/Seconds]*/
    yarp::sig::Vector m_trajectoryGenerationReferenceAcceleration; /**< reference acceleration for trajectory generation in position mode. Currently NOT USED in trajectory generation! [Degrees/Seconds^2] */

    std::vector<std::string> m_jointNames;
    std::vector<std::string> controlboard_joint_names;
    std::vector<gazebo::physics::JointPtr> m_jointPointers; /* pointers for each joint, avoiding several calls to getJoint(joint_name) */

    typedef std::unordered_map<PidControlTypeEnum, std::vector<gazebo::common::PID>,
    yarp::dev::PidControlTypeEnumHashFunction> PIDMap;
    PIDMap m_pids;

    std::vector<gazebo::common::PID> m_impedancePosPDs;

    std::vector<std::string> m_position_control_law;
    std::vector<std::string> m_velocity_control_law;
    std::vector<std::string> m_impedance_control_law;
    std::vector<std::string> m_torque_control_law;

    yarp::sig::Vector m_torqueOffset;
    yarp::sig::Vector m_minStiffness;
    yarp::sig::Vector m_minDamping;
    yarp::sig::Vector m_maxStiffness;
    yarp::sig::Vector m_maxDamping;
    yarp::sig::Vector m_kPWM;

    double m_robotPositionToleranceRevolute {0.9};      // Degrees
    double m_robotPositionToleranceLinear   {0.004};    // Meters

    bool* m_isMotionDone;
    int * m_controlMode;
    int * m_interactionMode;

    bool m_started;
    int m_clock;
    int _T_controller;
    gazebo::common::Time m_previousTime;

    /**
     * Private methods
     */
    bool configureJointType();
    bool setMinMaxPos();  //NOT TESTED
    bool setMinMaxVel();
    bool setJointNames();  //WORKS
    bool setPIDsForGroup(std::string, PIDMap::mapped_type&, enum PIDFeedbackTerm pidTerms);
    bool setPIDsForGroup_POSITION(  std::vector<std::string>& control_law, PIDMap::mapped_type&);
    bool setPIDsForGroup_VELOCITY(  std::vector<std::string>& control_law, PIDMap::mapped_type&);
    bool setPIDsForGroup_IMPEDANCE( std::vector<std::string>& control_law, std::vector<gazebo::common::PID>&);
    bool setMinMaxImpedance();
    bool setPIDs(); //WORKS
    bool setMaxTorques();
    bool setPositionsToleranceRevolute();
    bool setPositionsToleranceLinear();

    bool findMotorControlGroup(yarp::os::Bottle& motorControlGroup_bot) const;

    bool check_joint_within_limits_override_torque(int i, double&ref );

    void resetAllPidsForJointAtIndex(int j);

    /**
     * \brief convert data read from Gazebo to user unit sistem,
     *  e.g. degrees for revolute joints and meters for prismatic joints
     * \param joint joint number
     * \param value value read from the Position method in Gazebo
     * \return value in user units
     */
    double convertGazeboToUser(int joint, double value);
    double *convertGazeboToUser(double *values);
    double convertGazeboGainToUserGain(int joint, double value);

    /**
     * \brief convert data read from user unit sistem to Gazebo one
     *  e.g. radiants for revolute joints and meters for prismatic joints
     * \param joint joint number
     * \param value Raw value read from Gazebo function like 'GetAngle'
     * \return value in Gazebo units (SI)
     */
    double convertUserToGazebo(int joint, double value);
    double convertUserGainToGazeboGain(int joint, double value);

    double convertUserToGazebo(double value);
    double *convertUserToGazebo(double *values);

    bool checkIfTorqueIsValid(const double* torques) const;
    bool checkIfTorqueIsValid(double torques) const;
};

#endif //GAZEBOYARP_CONTROLBOARDDRIVER_HH
