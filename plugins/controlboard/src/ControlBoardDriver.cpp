/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "ControlBoardDriver.h"
#include <GazeboYarpPlugins/common.h>

#include <cstdio>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/transport/transport.hh>

#include <yarp/os/LogStream.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;


GazeboYarpControlBoardDriver::GazeboYarpControlBoardDriver() : m_deviceName(""), m_initTime(true) {}

GazeboYarpControlBoardDriver::~GazeboYarpControlBoardDriver() {}


//generic function that check is key1 is present in input bottle and that the result has size elements
// return true/false
bool validate(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size)
{
    Bottle &tmp=input.findGroup(key1.c_str(), txt.c_str());
    if (tmp.isNull())
    {
        yError("%s not found\n", key1.c_str());
        return false;
    }
    if(tmp.size()!=size)
    {
        yError("%s incorrect number of entries\n", key1.c_str());
        return false;
    }
    out=tmp;
    return true;
}

bool GazeboYarpControlBoardDriver::gazebo_init()
{
    //m_robot = gazebo_pointer_wrapper::getModel();
    // yDebug()<<"if this message is the last one you read, m_robot has not been set";
    //assert is a NOP in release mode. We should change the error handling either with an exception or something else
    assert(m_robot);
    if (!m_robot) return false;

    if (!setJointNames()) return false;      // this function also fills in the m_jointPointers vector

    m_numberOfJoints = m_jointNames.size();

    m_positions.resize(m_numberOfJoints);
    m_motPositions.resize(m_numberOfJoints);
    m_zeroPosition.resize(m_numberOfJoints);
    m_jntReferenceVelocities.resize(m_numberOfJoints);
    m_velocities.resize(m_numberOfJoints);
    m_motVelocities.resize(m_numberOfJoints);
    m_amp.resize(m_numberOfJoints);
    m_torques.resize(m_numberOfJoints); m_torques.zero();
    m_measTorques.resize(m_numberOfJoints); m_measTorques.zero();
    m_maxTorques.resize(m_numberOfJoints, 2000.0);
    m_jntReferencePositions.resize(m_numberOfJoints);
    m_motReferencePositions.resize(m_numberOfJoints);
    m_motReferenceVelocities.resize(m_numberOfJoints);
    m_motReferenceTorques.resize(m_numberOfJoints);
    m_oldReferencePositions.resize(m_numberOfJoints);
    m_trajectoryGenerationReferencePosition.resize(m_numberOfJoints);
    m_trajectoryGenerationReferenceSpeed.resize(m_numberOfJoints);
    m_trajectoryGenerationReferenceAcceleration.resize(m_numberOfJoints);
    m_jntReferenceTorques.resize(m_numberOfJoints);
    m_jointPosLimits.resize(m_numberOfJoints);
    m_jointVelLimits.resize(m_numberOfJoints);
    m_impedancePosPDs.reserve(m_numberOfJoints);
    m_position_control_law.resize(m_numberOfJoints,"none");
    m_velocity_control_law.resize(m_numberOfJoints,"none");
    m_torque_control_law.resize(m_numberOfJoints,"none");
    m_impedance_control_law.resize(m_numberOfJoints,"none");
    m_torqueOffset.resize(m_numberOfJoints);
    m_minStiffness.resize(m_numberOfJoints, 0.0);
    m_maxStiffness.resize(m_numberOfJoints, 1000.0);
    m_minDamping.resize(m_numberOfJoints, 0.0);
    m_maxDamping.resize(m_numberOfJoints, 100.0);
    m_kPWM.resize(m_numberOfJoints,1.0);
    m_jointTypes.resize(m_numberOfJoints);
    m_positionThreshold.resize(m_numberOfJoints);

    // Prepare PID structures
    m_pids[VOCAB_PIDTYPE_POSITION].reserve(m_numberOfJoints);
    m_pids[VOCAB_PIDTYPE_VELOCITY].reserve(m_numberOfJoints);
    m_pids[VOCAB_PIDTYPE_TORQUE].reserve(m_numberOfJoints);

    // Initial zeroing of all vectors
    m_positions.zero();
    m_motPositions.zero();
    m_zeroPosition.zero();
    m_velocities.zero();
    m_motVelocities.zero();
    m_motReferencePositions.zero();
    m_motReferenceVelocities.zero();
    m_motReferenceTorques.zero();
    m_jntReferencePositions.zero();
    m_jntReferenceVelocities.zero();
    m_jntReferenceTorques.zero();
    m_trajectoryGenerationReferencePosition.zero();
    m_trajectoryGenerationReferenceSpeed.zero();
    m_trajectoryGenerationReferenceAcceleration.zero();
    m_amp = 1;
    m_controlMode = new int[m_numberOfJoints];
    m_interactionMode = new int[m_numberOfJoints];
    m_isMotionDone = new bool[m_numberOfJoints];
    m_clock = 0;
    m_torqueOffset = 0;
    m_initTime = true; // Set to initialize the simulation time to Gazebo simTime on the first call to onUpdate().

    m_trajectory_generator.resize(m_numberOfJoints, NULL);
    m_coupling_handler.clear();
    m_speed_ramp_handler.resize(m_numberOfJoints, NULL);
    m_velocity_watchdog.resize(m_numberOfJoints, NULL);

    m_useVirtualAnalogSensor = m_pluginParameters.check("useVirtualAnalogSensor", yarp::os::Value(false)).asBool();

    VectorOf<int> trajectory_generator_type;
    trajectory_generator_type.resize(m_numberOfJoints, yarp::dev::TRAJECTORY_TYPE_MIN_JERK);

    yarp::os::Bottle& traj_bottle = m_pluginParameters.findGroup("TRAJECTORY_GENERATION");
    if (!traj_bottle.isNull())
    {
        yarp::os::Bottle& traj_type = traj_bottle.findGroup("trajectory_type");
        if (!traj_type.isNull())
        {
            std::string traj_type_s = traj_type.get(1).asString();
            if      (traj_type_s == "constant_speed")    {for (size_t i = 0; i < m_numberOfJoints; ++i) {trajectory_generator_type[i] = yarp::dev::TRAJECTORY_TYPE_CONST_SPEED;}}
            else if (traj_type_s == "trapezoidal_speed") {for (size_t i = 0; i < m_numberOfJoints; ++i) {trajectory_generator_type[i] = yarp::dev::TRAJECTORY_TYPE_TRAP_SPEED;}}
            else if (traj_type_s == "minimum_jerk")      {/* default */}
            else                                         {yError() << "Unsupported trajectory_type:" << traj_type_s; return false;}
            yDebug() << "trajectory_type:" << traj_type_s;
        }
        else
        {
            yWarning() << "Missing TRAJECTORY_GENERATION group. Missing trajectory_type param. Assuming minimum_jerk";
        }
    }
    else
    {
        yWarning() << "Missing trajectory_type param. Assuming minimum_jerk";
    }

    for (size_t j = 0; j < m_numberOfJoints; ++j)
    {
        switch (trajectory_generator_type[j])
        {
        case yarp::dev::TRAJECTORY_TYPE_MIN_JERK:
            m_trajectory_generator[j] = new MinJerkTrajectoryGenerator(m_robot);
            break;
        case yarp::dev::TRAJECTORY_TYPE_CONST_SPEED:
            m_trajectory_generator[j] = new ConstSpeedTrajectoryGenerator(m_robot);
            break;
        case yarp::dev::TRAJECTORY_TYPE_TRAP_SPEED:
            m_trajectory_generator[j] = new TrapezoidalSpeedTrajectoryGenerator(m_robot);
            break;
        }
    }

    for (size_t j = 0; j < m_numberOfJoints; ++j)
    {
        m_speed_ramp_handler[j] = new RampFilter();
        m_velocity_watchdog[j] = new Watchdog(0.200); //watchdog set to 200ms
    }

    yDebug() << "done";
    for (size_t j = 0; j < m_numberOfJoints; ++j)
    {
        m_controlMode[j] = VOCAB_CM_POSITION;
        m_interactionMode[j] = VOCAB_IM_STIFF;
        m_jointTypes[j] = JointType_Unknown;
        m_isMotionDone[j] = true;
    }
    // End zeroing of vectors

    // This must be after zeroing of vectors
    if(!configureJointType() )
        return false;

    if (!setMinMaxPos())
    {
        yError()<<"Failed to get joint limits";
        return false;
    }

    // This must be after calling setMinMaxPos as we need to access m_jointPosLimits variables
    // once they are initialized
    yarp::os::Bottle& coupling_group_bottle = m_pluginParameters.findGroup("COUPLING");
    if (!coupling_group_bottle.isNull())
    {
        if (coupling_group_bottle.size() ==0)
        {
            yError() << "Missing param in COUPLING section";
            return false;
        }
        yDebug() << "Requested couplings:" << coupling_group_bottle.toString();
        yDebug() << "Size: " << coupling_group_bottle.size();

        for (int cnt=1; cnt<coupling_group_bottle.size(); cnt++)
        {
            yarp::os::Bottle* coupling_bottle = coupling_group_bottle.get(cnt).asList();

            if (coupling_bottle == 0 || (coupling_bottle->size() != 3 && coupling_bottle->size() != 5))
            {
                yError() << "Error parsing coupling parameter"; return false;
            }
            //yDebug() << "Requested coupling:" << cnt << " / " << coupling_bottle->size();
            //yDebug() << "Requested coupling:" << coupling_bottle->toString();

            yarp::sig::VectorOf<int> coupled_joints;
            std::vector<std::string> coupled_joint_names;
            Bottle* b = coupling_bottle->get(1).asList();
            if (b==0 || b->size()==0) {
                yError() << "Error parsing coupling parameter, wrong size of the joints numbers list";
                return false;
            }
            for (int is=0;is<b->size();is++) {
                coupled_joints.push_back(b->get(is).asInt32());
            }
            Bottle* b2 = coupling_bottle->get(2).asList();
            if (b2==0 || b2->size()==0)
            {
                yError() << "Error parsing coupling parameter, wrong size of the joint names list";
                return false;
            }
            std::size_t number_coupled_joints = 0;
            for (int is=0;is<b2->size();is++) {
                const std::string& joint_name = b2->get(is).asString();
                coupled_joint_names.push_back(joint_name);

                if (joint_name != "reserved")
                    number_coupled_joints++;
            }

            // Fill limits for coupled joints
            std::vector<Range> coupled_joint_limits;
            if (coupling_bottle->size() == 5)
            {
                // Min limits of coupled joints.
                Bottle* b3 = coupling_bottle->get(3).asList();
                if (b3==0 || b3->size()!=number_coupled_joints)
                {
                    yError() << "Error parsing coupling parameter, wrong size of the joint min limits list";
                    return false;
                }

                // Max limits of coupled joints.
                Bottle* b4 = coupling_bottle->get(4).asList();
                if (b4==0 || b4->size()!=number_coupled_joints)
                {
                    yError() << "Error parsing coupling parameter, wrong size of the joint max limits list";
                    return false;
                }

                // Populate vector of limits
                for (std::size_t i=0; i<b3->size(); i++)
                {
                    Range range;
                    range.min = b3->get(i).asFloat64();
                    range.max = b4->get(i).asFloat64();

                    coupled_joint_limits.push_back(range);
                }
            }
            else
            {
                yWarning() << "The coupling handler entry does not specify coupled limits. The limits available in the [LIMITS] section will be employed instead. Please consider filling up the limits section in the coupling handler to avoid misbheaviors.";

                for (unsigned int i = 0; i < coupled_joints.size(); ++i)
                    coupled_joint_limits.push_back(m_jointPosLimits[coupled_joints[i]]);
            }

            if (coupling_bottle->get(0).asString()=="eyes_vergence_control")
            {
                BaseCouplingHandler* cpl = new EyesCouplingHandler(m_robot,coupled_joints, coupled_joint_names, coupled_joint_limits);
                m_coupling_handler.push_back(cpl);
                yInfo() << "using eyes_vergence_control";
            }
            else if (coupling_bottle->get(0).asString()=="fingers_abduction_control")
            {
                BaseCouplingHandler* cpl = new FingersAbductionCouplingHandler(m_robot,coupled_joints, coupled_joint_names, coupled_joint_limits);
                m_coupling_handler.push_back(cpl);
                yInfo() << "using fingers_abduction_control";
            }
            else if (coupling_bottle->get(0).asString()=="thumb_control")
            {
                BaseCouplingHandler* cpl = new ThumbCouplingHandler(m_robot,coupled_joints, coupled_joint_names, coupled_joint_limits);
                m_coupling_handler.push_back(cpl);
                yInfo() << "using thumb_control";
            }
            else if (coupling_bottle->get(0).asString()=="index_control")
            {
                BaseCouplingHandler* cpl = new IndexCouplingHandler(m_robot,coupled_joints, coupled_joint_names, coupled_joint_limits);
                m_coupling_handler.push_back(cpl);
                yInfo() << "using index_control";
            }
            else if (coupling_bottle->get(0).asString()=="middle_control")
            {
                BaseCouplingHandler* cpl = new MiddleCouplingHandler(m_robot,coupled_joints, coupled_joint_names, coupled_joint_limits);
                m_coupling_handler.push_back(cpl);
                yInfo() << "using middle_control";
            }
            else if (coupling_bottle->get(0).asString()=="pinky_control")
            {
                BaseCouplingHandler* cpl = new PinkyCouplingHandler(m_robot,coupled_joints, coupled_joint_names, coupled_joint_limits);
                m_coupling_handler.push_back(cpl);
                yInfo() << "using pinky_control";
            }
            else if (coupling_bottle->get(0).asString()=="cer_hand")
            {
                BaseCouplingHandler* cpl = new CerHandCouplingHandler(m_robot,coupled_joints, coupled_joint_names, coupled_joint_limits);
                m_coupling_handler.push_back(cpl);
                yInfo() << "using cer_hand_control";
            }
            else if (coupling_bottle->get(0).asString()=="icub_hand_mk3")
            {
                BaseCouplingHandler* cpl = new HandMk3CouplingHandler(m_robot,coupled_joints, coupled_joint_names, coupled_joint_limits);
                m_coupling_handler.push_back(cpl);
                yInfo() << "using icub_hand_mk3";
            }
            else if (coupling_bottle->get(0).asString()=="none")
            {
                yDebug() << "Just for test";
            }
            else
            {
                yError() << "Unknown coupling type";
                return false;
            }

        }
    }

    if (!setMinMaxVel())
    {
        yError()<<"Failed to get Velocity Limits";
        //return false; //to be added soon
    }

    // NOTE: This has to be after setMinMaxVel function
    if (!setTrajectoryReferences())
    {
        yError()<<"Failed to get Trajectory References";
        return false;
    }

    if (!setPositionsToleranceLinear())
    {
        yError()<<"Failed PositionsToleranceLinear initialization";
        return false;
    }

    if (!setPositionsToleranceRevolute())
    {
        yError()<<"Failed PositionsToleranceRevolute initialization";
        return false;
    }

    if (!setMaxTorques())
    {
        yError()<<"Failed Max Torque initialization";
        return false;
    }

    if (!setMinMaxImpedance())
    {
        yError()<<"Failed Impedance initialization";
        return false;
    }

    if (!setPIDs())
    {
        yError()<<"Failed PID initialization";
        return false;
    }

    // Connect the onUpdate method to the WorldUpdateBegin event callback
    this->m_updateConnection =
    gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboYarpControlBoardDriver::onUpdate,
                                                               this, _1));

    // Connect the onReset method to the WorldReset event callback
    this->m_resetConnection =
    gazebo::event::Events::ConnectWorldReset(boost::bind(&GazeboYarpControlBoardDriver::onReset, this));

    _T_controller = 1;

    resetPositionsAndTrajectoryGenerators();

    return true;
}

void GazeboYarpControlBoardDriver::resetPositionsAndTrajectoryGenerators()
{
    if(m_pluginParameters.check("initialConfiguration") )
    {
        std::stringstream ss(m_pluginParameters.find("initialConfiguration").toString());
        double tmp = 0.0;
        yarp::sig::Vector initial_config(m_numberOfJoints);
        unsigned int counter = 1;
        while (ss >> tmp) {
            if(counter > m_numberOfJoints) {
                yError()<<"Too many element in initial configuration, stopping at element "<<counter;
                break;
            }
            initial_config[counter-1] = tmp;
            m_trajectoryGenerationReferencePosition[counter - 1] = convertGazeboToUser(counter-1, tmp);
            m_jntReferencePositions[counter - 1] = convertGazeboToUser(counter-1, tmp);
            m_positions[counter - 1] = convertGazeboToUser(counter-1, tmp);
            counter++;
        }
        yDebug()<<"INITIAL CONFIGURATION IS: "<<initial_config.toString();

        // Set initial reference
        for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
            m_jointPointers[i]->SetPosition(0,initial_config[i]);
        }

        yDebug() << "Initializing Trajectory Generator with default values";
        for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
            if (isValidUserDOF(i)) {

                double limit_min, limit_max;
                getUserDOFLimit(i, limit_min, limit_max);
                m_trajectory_generator[i]->setLimits(limit_min, limit_max);

                m_trajectory_generator[i]->initTrajectory(m_positions[i],
                                                          m_positions[i],
                                                          m_trajectoryGenerationReferenceSpeed[i],
                                                          m_trajectoryGenerationReferenceAcceleration[i]);
            }
        }
    }
    else
    {
        yDebug() << "Initializing Trajectory Generator with current values";
        yarp::sig::Vector initial_positions;
        for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
#if GAZEBO_MAJOR_VERSION >= 8
                double gazeboPos = m_jointPointers[i]->Position(0);
#else
                double gazeboPos = m_jointPointers[i]->GetAngle(0).Radian();
#endif
                initial_positions.push_back(convertGazeboToUser(i, gazeboPos));
        }

        for (size_t cpl_cnt = 0; cpl_cnt < m_coupling_handler.size(); cpl_cnt++) {
            if (m_coupling_handler[cpl_cnt])
                m_coupling_handler[cpl_cnt]->decouplePos(initial_positions);
        }

        for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
            if (isValidUserDOF(i)) {

                double limit_min, limit_max;
                getUserDOFLimit(i, limit_min, limit_max);
                m_trajectory_generator[i]->setLimits(limit_min, limit_max);

                m_trajectory_generator[i]->initTrajectory(initial_positions[i],
                                                          initial_positions[i],
                                                          m_trajectoryGenerationReferenceSpeed[i],
                                                          m_trajectoryGenerationReferenceAcceleration[i]);
            }
        }
    }

    for (unsigned int j = 0; j < m_numberOfJoints; ++j)
    {
        m_controlMode[j] = VOCAB_CM_POSITION;
        m_interactionMode[j] = VOCAB_IM_STIFF;
    }

    for (unsigned int j = 0; j < m_numberOfJoints; ++j)
    {
        // Set an old reference value surely out of range. This will force the setting of initial reference at startup
        // NOTE: This has to be after setMinMaxPos function
        m_oldReferencePositions[j] = 1e21;
    }
}


bool GazeboYarpControlBoardDriver::configureJointType()
{
    bool ret = true;
    //////// determine the type of joint (rotational or prismatic)
    for(size_t i = 0; i < m_numberOfJoints; ++i)
    {
        switch (m_jointPointers[i]->GetType())
        {
            case ( gazebo::physics::Entity::HINGE_JOINT  |  gazebo::physics::Entity::JOINT):
            {
                m_jointTypes[i] = JointType_Revolute;
                m_positionThreshold[i] = m_robotPositionToleranceRevolute;
                break;
            }

            case ( gazebo::physics::Entity::SLIDER_JOINT |  gazebo::physics::Entity::JOINT):
            {
                m_jointTypes[i] = JointType_Prismatic;
                m_positionThreshold[i] = m_robotPositionToleranceLinear;
                break;
            }

            default:
            {
                yError() << "joint type is not supported by Gazebo YARP plugin now. Supported joint types are 'revolute' and 'prismatic' \n\t(GEARBOX_JOINT and SLIDER_JOINT using Gazebo enums defined into gazebo/physic/base.hh include file, GetType() returns " << m_jointPointers[i]->GetType() ;
                m_jointTypes[i] = JointType_Unknown;
                ret = false;
                break;
            }
        }
    }
    return ret;
}

void GazeboYarpControlBoardDriver::onUpdate(const gazebo::common::UpdateInfo& _info)
{
    if (m_initTime) {
        m_initTime = false;
        m_previousTime = _info.simTime;
    }
    gazebo::common::Time stepTime = _info.simTime - m_previousTime;
    m_previousTime = _info.simTime;
    m_clock++;
    // measurements acquisition
    for (size_t jnt_cnt = 0; jnt_cnt < m_jointPointers.size(); jnt_cnt++)
    {
#if GAZEBO_MAJOR_VERSION >= 8
        double gazeboPos = m_jointPointers[jnt_cnt]->Position(0);
#else
        double gazeboPos = m_jointPointers[jnt_cnt]->GetAngle(0).Radian();
#endif
        m_positions[jnt_cnt] = convertGazeboToUser(jnt_cnt, gazeboPos);
        m_velocities[jnt_cnt] = convertGazeboToUser(jnt_cnt, m_jointPointers[jnt_cnt]->GetVelocity(0));
        if (!m_useVirtualAnalogSensor)
        {
            m_torques[jnt_cnt] = m_jointPointers[jnt_cnt]->GetForce(0u);
        }
        else
        {
            m_torques[jnt_cnt] = m_measTorques[jnt_cnt];
        }
    }

    m_motPositions=m_positions;
    m_motVelocities=m_velocities;
    //measurements decoupling
    for (size_t cpl_cnt = 0; cpl_cnt < m_coupling_handler.size(); cpl_cnt++)
    {
        if (m_coupling_handler[cpl_cnt])
        {
            m_coupling_handler[cpl_cnt]->decouplePos(m_positions);
            m_coupling_handler[cpl_cnt]->decoupleVel(m_velocities);
            //m_coupling_handler[cpl_cnt]->decoupleAcc(m_accelerations); //missing
            m_coupling_handler[cpl_cnt]->decoupleTrq(m_torques);
        }
    }

    // check measured torque for hw fault
    for (size_t jnt_cnt = 0; jnt_cnt < m_jointPointers.size(); jnt_cnt++) {
        if (m_controlMode[jnt_cnt]!=VOCAB_CM_HW_FAULT && fabs(m_torques[jnt_cnt])>m_maxTorques[jnt_cnt])
        {
            m_controlMode[jnt_cnt]=VOCAB_CM_HW_FAULT;
            yError() << "An hardware fault occurred on joint "<< jnt_cnt << " torque too big! ( " << m_torques[jnt_cnt] << " )";
        }
    }

    // Updating timestamp
    m_lastTimestamp.update(_info.simTime.Double());

    //update refernce m_jntReferenceVelocities
    for (size_t j = 0; j < m_numberOfJoints; ++j)
    {
        if (m_speed_ramp_handler[j])
        {
            if (m_velocity_watchdog[j]->isExpired())
            {
                m_speed_ramp_handler[j]->stop();
            }
            m_speed_ramp_handler[j]->update();
            //yDebug() << m_speed_ramp_handler[j]->getCurrentValue();
        }
    }

    //update Trajectories
    for (size_t j = 0; j < m_numberOfJoints; ++j)
    {
        if (m_controlMode[j] == VOCAB_CM_POSITION)
        {
            if (m_clock % _T_controller == 0)
            {
                m_jntReferencePositions[j] = m_trajectory_generator[j]->computeTrajectory();
                m_isMotionDone[j] = m_trajectory_generator[j]->isMotionDone();
            }
        }
        else if (m_controlMode[j] == VOCAB_CM_MIXED)
        {
            if (m_clock % _T_controller == 0)
            {
                double computed_ref_speed = m_speed_ramp_handler[j]->getCurrentValue()*stepTime.Double();  //controller period
                double computed_ref_pos =  m_jntReferencePositions[j] + m_trajectory_generator[j]->computeTrajectoryStep();
                m_jntReferencePositions[j] = computed_ref_pos + computed_ref_speed;
                //yDebug() << computed_ref_pos << " " << computed_ref_speed;
                m_isMotionDone[j] = m_trajectory_generator[j]->isMotionDone();
            }
        }
        else if (m_controlMode[j] == VOCAB_CM_VELOCITY)
        {
            if (m_clock % _T_controller == 0)
            {
                if (m_speed_ramp_handler[j]) m_jntReferenceVelocities[j] = m_speed_ramp_handler[j]->getCurrentValue();
            }

            if (m_velocity_control_type == IntegratorAndPositionPID)
            {
                // All quantities are in degrees for revolute, and meters for linear
                m_jntReferencePositions[j] = m_jntReferencePositions[j] + m_speed_ramp_handler[j]->getCurrentValue()*stepTime.Double();
            }
        }
    }

    //references decoupling
    m_motReferencePositions=m_jntReferencePositions;
    m_motReferenceVelocities=m_jntReferenceVelocities;
    m_motReferenceTorques=m_jntReferenceTorques;
    for (size_t cpl_cnt = 0; cpl_cnt < m_coupling_handler.size(); cpl_cnt++)
    {
        if (m_coupling_handler[cpl_cnt])
        {
            m_motReferencePositions = m_coupling_handler[cpl_cnt]->decoupleRefPos(m_jntReferencePositions);
            m_motReferenceVelocities = m_coupling_handler[cpl_cnt]->decoupleRefVel(m_jntReferenceVelocities);
            m_motReferenceTorques = m_coupling_handler[cpl_cnt]->decoupleRefTrq(m_jntReferenceTorques);
        }
    }

    //update References
    for (size_t j = 0; j < m_numberOfJoints; ++j) {
        double forceReference = 0;

        //set pos joint value, set m_referenceVelocities joint value
        if ((m_controlMode[j] == VOCAB_CM_POSITION || m_controlMode[j] == VOCAB_CM_POSITION_DIRECT || (m_controlMode[j] == VOCAB_CM_VELOCITY && m_velocity_control_type == IntegratorAndPositionPID))      && (m_interactionMode[j] == VOCAB_IM_STIFF))
        {
            gazebo::common::PID &pid = m_pids[VOCAB_PIDTYPE_POSITION][j];
            forceReference = pid.Update(convertUserToGazebo(j, m_motPositions[j]) - convertUserToGazebo(j, m_motReferencePositions[j]), stepTime);
        }
        else if ((m_controlMode[j] == VOCAB_CM_POSITION || m_controlMode[j] == VOCAB_CM_POSITION_DIRECT || (m_controlMode[j] == VOCAB_CM_VELOCITY && m_velocity_control_type == IntegratorAndPositionPID)) && (m_interactionMode[j] == VOCAB_IM_COMPLIANT))
        {
            double q = m_motPositions[j] - m_zeroPosition[j];
            forceReference  = -m_impedancePosPDs[j].GetPGain() * (q - m_motReferencePositions[j]) - m_impedancePosPDs[j].GetDGain() * m_motVelocities[j] + m_torqueOffset[j];
        }
        else if ((m_controlMode[j] == VOCAB_CM_VELOCITY && m_velocity_control_type == DirectVelocityPID) && (m_interactionMode[j] == VOCAB_IM_STIFF))
        {
            gazebo::common::PID &pid = m_pids[VOCAB_PIDTYPE_VELOCITY][j];
            forceReference = pid.Update(convertUserToGazebo(j, m_motVelocities[j]) - convertUserToGazebo(j, m_motReferenceVelocities[j]), stepTime);
        }
        else if ((m_controlMode[j] == VOCAB_CM_VELOCITY && m_velocity_control_type == DirectVelocityPID) && (m_interactionMode[j] == VOCAB_IM_COMPLIANT))
        {
            gazebo::common::PID &pid = m_pids[VOCAB_PIDTYPE_VELOCITY][j];
            forceReference = pid.Update(convertUserToGazebo(j, m_motVelocities[j]) - convertUserToGazebo(j, m_motReferenceVelocities[j]), stepTime);
            yWarning("Compliant velocity control not yet implemented");
        }
        else if ((m_controlMode[j] == VOCAB_CM_MIXED) && (m_interactionMode[j] == VOCAB_IM_STIFF))
        {
            gazebo::common::PID &pid = m_pids[VOCAB_PIDTYPE_POSITION][j];
            forceReference = pid.Update(convertUserToGazebo(j, m_motPositions[j]) - convertUserToGazebo(j, m_motReferencePositions[j]), stepTime);

        }
        else if ((m_controlMode[j] == VOCAB_CM_MIXED) && (m_interactionMode[j] == VOCAB_IM_COMPLIANT))
        {
            double q = m_motPositions[j] - m_zeroPosition[j];
            forceReference  = -m_impedancePosPDs[j].GetPGain() * (q - m_motReferencePositions[j]) - m_impedancePosPDs[j].GetDGain() * m_motVelocities[j] + m_torqueOffset[j];
        }
        else if (m_controlMode[j] == VOCAB_CM_TORQUE)
        {
            forceReference = m_motReferenceTorques[j];
        }
        else if (m_controlMode[j] == VOCAB_CM_IDLE)
        {
            forceReference = 0;
        }
        else if (m_controlMode[j] == VOCAB_CM_HW_FAULT)
        {
            forceReference = 0;
        }
        else if (m_controlMode[j] == VOCAB_CM_PWM)
        {
            //PWM control sends torques to gazebo at this moment.
            //Check if gazebo implements a "motor" entity and change the code accordingly.'
            forceReference = m_motReferenceTorques[j];
        }
        else if (m_controlMode[j] == VOCAB_CM_CURRENT)
        {
            //PWM control sends torques to gazebo at this moment.
            //Check if gazebo implements a "motor" entity and change the code accordingly.
            forceReference = m_motReferenceTorques[j];
        }
        if (check_joint_within_limits_override_torque(j, forceReference)==false) {}
        m_jointPointers[j]->SetForce(0, forceReference);
    }
}

void GazeboYarpControlBoardDriver::onReset()
{
    m_previousTime = gazebo::common::Time::Zero;
    m_initTime = true;
    resetPositionsAndTrajectoryGenerators();
}


bool GazeboYarpControlBoardDriver::setMinMaxPos()
{
    for (size_t i = 0; i < m_numberOfJoints; ++i)
    {
#if GAZEBO_MAJOR_VERSION >= 8
        m_jointPosLimits[i].max = convertGazeboToUser(i, m_jointPointers[i]->UpperLimit(0));
        m_jointPosLimits[i].min = convertGazeboToUser(i, m_jointPointers[i]->LowerLimit(0));
#else
        m_jointPosLimits[i].max = convertGazeboToUser(i, m_jointPointers[i]->GetUpperLimit(0).Radian());
        m_jointPosLimits[i].min = convertGazeboToUser(i, m_jointPointers[i]->GetLowerLimit(0).Radian());
#endif
    }

    //...if the yarp plugin configuration file specifies a different velocity, override it...
    yarp::os::Bottle& limits_bottle = m_pluginParameters.findGroup("LIMITS");
    if (!limits_bottle.isNull())
    {
        yarp::os::Bottle& pos_limit_max = limits_bottle.findGroup("jntPosMax");
        if (!pos_limit_max.isNull() && static_cast<size_t>(pos_limit_max.size()) == m_numberOfJoints+1)
        {
            for(size_t i = 0; i < m_numberOfJoints; ++i)
            {
                m_jointPosLimits[i].max = pos_limit_max.get(i+1).asFloat64();

            }
        }
        else
        {
            yError() << "Failed to parse jntPosMax parameter";
            return false;
        }
        yarp::os::Bottle& pos_limit_min = limits_bottle.findGroup("jntPosMin");
        if (!pos_limit_min.isNull() && static_cast<size_t>(pos_limit_min.size()) == m_numberOfJoints+1)
        {
            for(size_t i = 0; i < m_numberOfJoints; ++i)
            {
                m_jointPosLimits[i].min = pos_limit_min.get(i+1).asFloat64();
            }
        }
        else
        {
            yError() << "Failed to parse jntPosMin parameter";
            return false;
        }
    }
    else
    {
        yWarning() << "Missing LIMITS section";
    }

    return true;
}

bool GazeboYarpControlBoardDriver::setMinMaxVel()
{
    //first check in gazebo...
    for(size_t i = 0; i < m_numberOfJoints; ++i)
    {
        m_jointVelLimits[i].max = convertGazeboToUser(i, m_jointPointers[i]->GetVelocityLimit(0));
        m_jointVelLimits[i].min = 0;
    }

    //...if the yarp plugin configuration file specifies a different velocity, override it...
    yarp::os::Bottle& limits_bottle = m_pluginParameters.findGroup("LIMITS");
    if (!limits_bottle.isNull())
    {
        yarp::os::Bottle& vel_limits = limits_bottle.findGroup("jntVelMax");
        if (!vel_limits.isNull())
        {
            for(size_t i = 0; i < m_numberOfJoints; ++i)
            {
                m_jointVelLimits[i].max = vel_limits.get(i+1).asFloat64();
                m_jointVelLimits[i].min = 0;
            }
        }
        else
        {
            yError() << "Failed to parse jntVelMax parameter";
            return false;
        }
    }
    else
    {
        yWarning() << "Missing LIMITS section";
    }
    return true;
}

bool GazeboYarpControlBoardDriver::setTrajectoryReferences()
{
    // set sensible defaults
    for (size_t j = 0; j < m_numberOfJoints; ++j)
    {
        if (m_jointTypes[j] == JointType_Revolute)
        {
            m_trajectoryGenerationReferenceSpeed[j] = 10.0; // deg/s
            m_trajectoryGenerationReferenceAcceleration[j] = 10.0; // deg/s^2
        }
        else if (m_jointTypes[j] == JointType_Prismatic)
        {
            m_trajectoryGenerationReferenceSpeed[j] = 0.010; // m/s
            m_trajectoryGenerationReferenceAcceleration[j] = 0.010; // m/s^2
        }
    }

    // pick user values, if any
    yarp::os::Bottle& traj_bottle = m_pluginParameters.findGroup("TRAJECTORY_GENERATION");

    if (!traj_bottle.isNull())
    {
        yarp::os::Bottle& refSpeeds = traj_bottle.findGroup("refSpeed");

        if (!refSpeeds.isNull())
        {
            if (static_cast<size_t>(refSpeeds.size()) - 1 == m_numberOfJoints)
            {
                for (size_t j = 0; j < m_numberOfJoints; ++j)
                {
                    m_trajectoryGenerationReferenceSpeed[j] = refSpeeds.get(j + 1).asFloat64();
                }
            }
            else
            {
                yError() << "Invalid number of refSpeed params";
                return false;
            }
        }

        yarp::os::Bottle& refAccelerations = traj_bottle.findGroup("refAcceleration");

        if (!refAccelerations.isNull())
        {
            if (static_cast<size_t>(refAccelerations.size()) - 1 == m_numberOfJoints)
            {
                for (size_t j = 0; j < m_numberOfJoints; ++j)
                {
                    m_trajectoryGenerationReferenceAcceleration[j] = refAccelerations.get(j + 1).asFloat64();
                }
            }
            else
            {
                yError() << "Invalid number of refAcceleration params";
                return false;
            }
        }
    }

    // clip values according to joint limits
    for (size_t j = 0; j < m_numberOfJoints; ++j)
    {
        if (m_trajectoryGenerationReferenceSpeed[j] > m_jointVelLimits[j].max)
        {
            m_trajectoryGenerationReferenceSpeed[j] = m_jointVelLimits[j].max;
        }
    }

    yDebug() << "refSpeed: [ " << m_trajectoryGenerationReferenceSpeed.toString() << " ] ";
    yDebug() << "refAcceleration: [ " << m_trajectoryGenerationReferenceAcceleration.toString() << " ] ";

    return true;
}

bool GazeboYarpControlBoardDriver::setJointNames()  //WORKS
{
    yarp::os::Bottle joint_names_bottle = m_pluginParameters.findGroup("jointNames");

    if (joint_names_bottle.isNull()) {
        yError() << "GazeboYarpControlBoardDriver::setJointNames(): Error cannot find jointNames." ;
        return false;
    }

    int nr_of_joints = joint_names_bottle.size()-1;

    m_jointNames.resize(nr_of_joints);
    m_jointPointers.resize(nr_of_joints);

    const gazebo::physics::Joint_V & gazebo_models_joints = m_robot->GetJoints();

    controlboard_joint_names.clear();
    for (size_t i = 0; i < m_jointNames.size(); i++) {
        bool joint_found = false;
        controlboard_joint_names.push_back(joint_names_bottle.get(i+1).asString().c_str());

        for (size_t gazebo_joint = 0; gazebo_joint < gazebo_models_joints.size() && !joint_found; gazebo_joint++) {
            std::string gazebo_joint_name = gazebo_models_joints[gazebo_joint]->GetName();
            if (GazeboYarpPlugins::hasEnding(gazebo_joint_name,controlboard_joint_names[i])) {
                joint_found = true;
                m_jointNames[i] = gazebo_joint_name;
                m_jointPointers[i] = this->m_robot->GetJoint(gazebo_joint_name);
                yDebug() << "found: " <<  gazebo_joint_name << controlboard_joint_names[i];
            }
        }

        if (!joint_found) {
            yError() << "GazeboYarpControlBoardDriver::setJointNames(): cannot find joint '" << controlboard_joint_names[i]
            << "' (" << i+1 << " of " << nr_of_joints << ") " << "\n";
            yError() << "jointNames are " << joint_names_bottle.toString() << "\n";
            m_jointNames.resize(0);
            m_jointPointers.resize(0);
            return false;
        }
    }
    return true;
}

bool GazeboYarpControlBoardDriver::setPIDsForGroup(std::string pidGroupName,
                                                   PIDMap::mapped_type& pids,
                                                   enum PIDFeedbackTerm pidTerms)
{
    yarp::os::Property prop;
    if (m_pluginParameters.check(pidGroupName.c_str())) {
        for (size_t i = 0; i < m_numberOfJoints; ++i) {
            std::stringstream property_name;
            property_name<<"Pid";
            property_name<<i;

            yarp::os::Bottle& pid = m_pluginParameters.findGroup(pidGroupName.c_str()).findGroup(property_name.str().c_str());

            gazebo::common::PID pidValue;
            if (pidTerms & PIDFeedbackTermProportionalTerm)
                pidValue.SetPGain(pid.get(1).asFloat64());
            if (pidTerms & PIDFeedbackTermDerivativeTerm)
                pidValue.SetDGain(pid.get(2).asFloat64());
            if (pidTerms & PIDFeedbackTermIntegrativeTerm)
                pidValue.SetIGain(pid.get(3).asFloat64());

            pidValue.SetIMax(pid.get(4).asFloat64());
            pidValue.SetIMin(-pid.get(4).asFloat64());
            pidValue.SetCmdMax(pid.get(5).asFloat64());
            pidValue.SetCmdMin(-pid.get(5).asFloat64());

            pids.push_back(pidValue);
        }
    } else {
        double default_p = pidTerms & PIDFeedbackTermProportionalTerm ? 500.0 : 0;
        double default_i = pidTerms & PIDFeedbackTermIntegrativeTerm ? 0.1 : 0;
        double default_d = pidTerms & PIDFeedbackTermDerivativeTerm ? 1.0 : 0;
        yWarning()<<"PID gain information not found in group " << pidGroupName << ", using default gains ( "
        <<"P " << default_p << " I " << default_i << " D " << default_d << " )";
        for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
            gazebo::common::PID pid(500, 0.1, 1.0, 0.0, 0.0);
            pids.push_back(pid);
        }
    }

    return true;
}

bool GazeboYarpControlBoardDriver::setPIDsForGroup_POSITION(std::vector<std::string>& control_law, PIDMap::mapped_type& pids )
{
    yarp::os::Property prop;
    if (m_pluginParameters.check("POSITION_CONTROL"))
    {
        Bottle xtmp;
        Bottle pidGroup = m_pluginParameters.findGroup("POSITION_CONTROL");

        //control units block
        enum units_type {metric=0, si=1} c_units=metric;
        xtmp = pidGroup.findGroup("controlUnits");
        if (!xtmp.isNull())
        {
            if      (xtmp.get(1).asString()==std::string("metric_units"))  {c_units=metric;}
            else if (xtmp.get(1).asString()==std::string("si_units"))      {c_units=si;}
            else    {yError() << "invalid controlUnits value"; return false;}
        }
        else
        {
            yError ("POSITION_CONTROL: 'controlUnits' param missing. Cannot continue");
            return false;
        }

        //control law block
        xtmp = pidGroup.findGroup("controlLaw");
        if (!xtmp.isNull())
        {
            if      (xtmp.get(1).asString()==std::string("joint_pid_gazebo_v1"))
            {
                for(unsigned int i=0; i<m_numberOfJoints; i++) control_law[i]="joint_pid_gazebo_v1";
            }
            else    {yError() << "invalid controlLaw value"; return false;}
        }
        else
        {
            yError ("POSITION_CONTROL: 'controlLaw' param missing. Cannot continue");
            return false;
        }

        std::vector<dev::Pid> yarpPid(m_numberOfJoints);

        bool error=false;
        size_t j=0;

        //control parameters
        if (!validate(pidGroup, xtmp, "kp", "Pid kp parameter", m_numberOfJoints+1)) {
            error = true;
        } else {
            for (j=0; j<m_numberOfJoints; j++) {
                yarpPid[j].kp = xtmp.get(j+1).asFloat64();
            }
        }
        if (!validate(pidGroup, xtmp, "kd", "Pid kd parameter", m_numberOfJoints+1)) {
            error=true;
        } else {
            for (j=0; j<m_numberOfJoints; j++) {
                yarpPid[j].kd = xtmp.get(j+1).asFloat64();
            }
        }
        if (!validate(pidGroup, xtmp, "ki", "Pid kp parameter", m_numberOfJoints+1)) {
            error=true;
        } else {
            for (j=0; j<m_numberOfJoints; j++) {
                yarpPid[j].ki = xtmp.get(j+1).asFloat64();
            }
        }
        if (!validate(pidGroup, xtmp, "maxInt", "Pid maxInt parameter", m_numberOfJoints+1)) {
            error=true;
        } else {
            for (j=0; j<m_numberOfJoints; j++) {
                yarpPid[j].max_int = xtmp.get(j+1).asFloat64();
            }
        }
        if (!validate(pidGroup, xtmp, "maxOutput", "Pid maxOutput parameter", m_numberOfJoints+1)) {
            error=true;
        } else {
            for (j=0; j<m_numberOfJoints; j++) {
                yarpPid[j].max_output = xtmp.get(j+1).asFloat64();
            }
        }
        if (!validate(pidGroup, xtmp, "shift", "Pid shift parameter", m_numberOfJoints+1)){
            error=true;
        } else {
            for (j=0; j<m_numberOfJoints; j++) {
                yarpPid[j].scale = xtmp.get(j+1).asFloat64();
            }
        }
        if (!validate(pidGroup, xtmp, "ko", "Pid ko parameter", m_numberOfJoints+1)){
            error = true;
        } else {
            for (j = 0; j < m_numberOfJoints; ++j) {
                yarpPid[j].offset = xtmp.get(j + 1).asFloat64();
            }
        }
        if (!validate(pidGroup, xtmp, "stictionUp", "Pid stictionUp", m_numberOfJoints+1)){
            error=true;
        } else {
            for (j = 0; j < m_numberOfJoints; j++) {
                yarpPid[j].stiction_up_val = xtmp.get(j+1).asFloat64();
            }
        }
        if (!validate(pidGroup, xtmp, "stictionDwn", "Pid stictionDwn", m_numberOfJoints+1)) {
            error=true;
        } else {
            for (j=0; j<m_numberOfJoints; j++) {
                yarpPid[j].stiction_down_val = xtmp.get(j+1).asFloat64();
            }
        }

        if (error)
        {
            return false;
        }
        else
        {
            for (j = 0; j < m_numberOfJoints; ++j)
            {
                gazebo::common::PID pidValue;
                if (c_units==metric)
                {
                    pidValue.SetPGain(convertUserGainToGazeboGain(j,yarpPid[j].kp)/pow(2,yarpPid[j].scale));
                    pidValue.SetIGain(convertUserGainToGazeboGain(j,yarpPid[j].ki)/pow(2,yarpPid[j].scale));
                    pidValue.SetDGain(convertUserGainToGazeboGain(j,yarpPid[j].kd)/pow(2,yarpPid[j].scale));
                }
                else if (c_units==si)
                {
                    pidValue.SetPGain(yarpPid[j].kp/pow(2,yarpPid[j].scale));
                    pidValue.SetIGain(yarpPid[j].ki/pow(2,yarpPid[j].scale));
                    pidValue.SetDGain(yarpPid[j].kd/pow(2,yarpPid[j].scale));
                }
                pidValue.SetIMax(yarpPid[j].max_int);
                pidValue.SetIMin(-yarpPid[j].max_int);
                pidValue.SetCmdMax(yarpPid[j].max_output);
                pidValue.SetCmdMin(-yarpPid[j].max_output);
                pids.push_back(pidValue);
            }
        }

        return true;
    }
    else
    {
        return false;
    }
}

bool GazeboYarpControlBoardDriver::setPIDsForGroup_VELOCITY(std::vector<std::string>& control_law, PIDMap::mapped_type& pids )
{
    yarp::os::Property prop;
    if (m_pluginParameters.check("VELOCITY_CONTROL"))
    {
        Bottle xtmp;
        Bottle pidGroup = m_pluginParameters.findGroup("VELOCITY_CONTROL");

        // Check velocityControlImplementationType value
        // If not present, default to direct_velocity_pid for now
        if (pidGroup.find("velocityControlImplementationType").isNull()) {
            yWarning("VELOCITY_CONTROL: 'velocityControlImplementationType' param missing. "
                     " Using the default value of 'direct_velocity_pid', but the parameter will be compulsory gazebo-yarp-plugins 4.");
        } else {
            std::string velocityControlImplementationType = pidGroup.find("velocityControlImplementationType").toString();

            if (velocityControlImplementationType == "direct_velocity_pid") {
                m_velocity_control_type = DirectVelocityPID;
            } else if(velocityControlImplementationType == "integrator_and_position_pid") {
                m_velocity_control_type = IntegratorAndPositionPID;
            } else {
               yError("VELOCITY_CONTROL: 'velocityControlImplementationType' param has unsupported value '%s'.", velocityControlImplementationType.c_str());
               return false;
            }
        }

        if (m_velocity_control_type == DirectVelocityPID) {
            // control units block
            enum units_type {metric=0, si=1} c_units=metric;
            xtmp = pidGroup.findGroup("controlUnits");
            if (!xtmp.isNull())
            {
                if      (xtmp.get(1).asString()==std::string("metric_units"))  {
                    c_units=metric;
                } else if (xtmp.get(1).asString()==std::string("si_units"))      {c_units=si;}
                else    {yError() << "invalid controlUnits value"; return false;}
            }
            else
            {
                yError ("VELOCITY_CONTROL: 'controlUnits' param missing. Cannot continue");
                return false;
            }

            //control law block
            xtmp = pidGroup.findGroup("controlLaw");
            if (!xtmp.isNull())
            {
                if      (xtmp.get(1).asString()==std::string("joint_pid_gazebo_v1"))
                {
                    for(unsigned int i=0; i<m_numberOfJoints; i++) control_law[i]="joint_pid_gazebo_v1";
                }
                else    {yError() << "invalid controlLaw value"; return false;}
            }
            else
            {
                yError ("VELOCITY_CONTROL: 'controlLaw' param missing. Cannot continue");
                return false;
            }

            std::vector<dev::Pid> yarpPid(m_numberOfJoints);
            bool error=false;
            size_t j=0;

            //control parameters
            if (!validate(pidGroup, xtmp, "kp", "Pid kp parameter", m_numberOfJoints+1))           {error=true;} else {for (j=0; j<m_numberOfJoints; j++) yarpPid[j].kp = xtmp.get(j+1).asFloat64();}
            if (!validate(pidGroup, xtmp, "kd", "Pid kd parameter", m_numberOfJoints+1))           {error=true;} else {for (j=0; j<m_numberOfJoints; j++) yarpPid[j].kd = xtmp.get(j+1).asFloat64();}
            if (!validate(pidGroup, xtmp, "ki", "Pid kp parameter", m_numberOfJoints+1))           {error=true;} else {for (j=0; j<m_numberOfJoints; j++) yarpPid[j].ki = xtmp.get(j+1).asFloat64();}
            if (!validate(pidGroup, xtmp, "maxInt", "Pid maxInt parameter", m_numberOfJoints+1))   {error=true;} else {for (j=0; j<m_numberOfJoints; j++) yarpPid[j].max_int = xtmp.get(j+1).asFloat64();}
            if (!validate(pidGroup, xtmp, "maxOutput", "Pid maxOutput parameter", m_numberOfJoints+1))   {error=true;} else {for (j=0; j<m_numberOfJoints; j++) yarpPid[j].max_output = xtmp.get(j+1).asFloat64();}
            if (!validate(pidGroup, xtmp, "shift", "Pid shift parameter", m_numberOfJoints+1))     {error=true;} else {for (j=0; j<m_numberOfJoints; j++) yarpPid[j].scale = xtmp.get(j+1).asFloat64();}
            if (!validate(pidGroup, xtmp, "ko", "Pid ko parameter", m_numberOfJoints+1))           {error=true;} else {for (j=0; j<m_numberOfJoints; j++) yarpPid[j].offset = xtmp.get(j+1).asFloat64();}

            if (error)
            {
                return false;
            }
            else
            {
                for (j = 0; j < m_numberOfJoints; ++j)
                {
                    gazebo::common::PID pidValue;
                    if (c_units==metric)
                    {
                        pidValue.SetPGain(convertUserGainToGazeboGain(j,yarpPid[j].kp)/pow(2,yarpPid[j].scale));
                        pidValue.SetIGain(convertUserGainToGazeboGain(j,yarpPid[j].ki)/pow(2,yarpPid[j].scale));
                        pidValue.SetDGain(convertUserGainToGazeboGain(j,yarpPid[j].kd)/pow(2,yarpPid[j].scale));
                    }
                    else if (c_units==si)
                    {
                        pidValue.SetPGain(yarpPid[j].kp/pow(2,yarpPid[j].scale));
                        pidValue.SetIGain(yarpPid[j].ki/pow(2,yarpPid[j].scale));
                        pidValue.SetDGain(yarpPid[j].kd/pow(2,yarpPid[j].scale));
                    }
                    pidValue.SetIMax(yarpPid[j].max_int);
                    pidValue.SetIMin(-yarpPid[j].max_int);
                    pidValue.SetCmdMax(yarpPid[j].max_output);
                    pidValue.SetCmdMin(-yarpPid[j].max_output);

                    pids.push_back(pidValue);
                }
            }
        }

        return true;
    }
    else
    {
        return false;
    }
}

bool GazeboYarpControlBoardDriver::setPIDsForGroup_IMPEDANCE(std::vector<std::string>& control_law, std::vector<gazebo::common::PID>& pids )
{
    yarp::os::Property prop;
    if (m_pluginParameters.check("IMPEDANCE_CONTROL"))
    {
        Bottle xtmp;
        Bottle pidGroup = m_pluginParameters.findGroup("IMPEDANCE_CONTROL");

        //control units block
        enum units_type {metric=0, si=1} c_units=metric;
        xtmp = pidGroup.findGroup("controlUnits");
        if (!xtmp.isNull())
        {
            if      (xtmp.get(1).asString()==std::string("metric_units"))  {c_units=metric;}
            else if (xtmp.get(1).asString()==std::string("si_units"))      {c_units=si;}
            else    {yError() << "invalid controlUnits value"; return false;}
        }
        else
        {
            yError ("IMPEDANCE_CONTROL: 'controlUnits' param missing. Cannot continue");
            return false;
        }

        //control law block
        xtmp = pidGroup.findGroup("controlLaw");
        if (!xtmp.isNull())
        {
            if      (xtmp.get(1).asString()==std::string("joint_pid_gazebo_v1"))
            {
                for(unsigned int i=0; i<m_numberOfJoints; i++) control_law[i]="joint_pid_gazebo_v1";
            }
            else    {yError() << "invalid controlLaw value"; return false;}
        }
        else
        {
            yError ("IMPEDANCE_CONTROL: 'controlLaw' param missing. Cannot continue");
            return false;
        }

        std::vector<double> stiffness(m_numberOfJoints);
        std::vector<double> damping(m_numberOfJoints);
        bool error=false;
        size_t j=0;

        //control parameters
        if (!validate(pidGroup, xtmp, "stiffness", "stiffness", m_numberOfJoints+1))    {error=true;} else {for (j=0; j<m_numberOfJoints; j++) stiffness[j] = xtmp.get(j+1).asFloat64();}
        if (!validate(pidGroup, xtmp, "damping", "damping", m_numberOfJoints+1))        {error=true;} else {for (j=0; j<m_numberOfJoints; j++) damping[j] = xtmp.get(j+1).asFloat64();}

        if (error)
        {
            return false;
        }
        else
        {
            for (j = 0; j < m_numberOfJoints; ++j)
            {
                gazebo::common::PID pidValue;
                if (c_units==metric)
                {
                    pidValue.SetPGain(convertUserGainToGazeboGain(j,stiffness[j]));
                    pidValue.SetDGain(convertUserGainToGazeboGain(j,damping[j]));
                }
                else if (c_units==si)
                {
                    pidValue.SetPGain(stiffness[j]);
                    pidValue.SetDGain(damping[j]);
                }
                pidValue.SetIGain(0);
                pidValue.SetIMax(0);
                pidValue.SetIMin(0);
                pidValue.SetCmdMax(0);
                pidValue.SetCmdMin(0);
                pids.push_back(pidValue);
            }
        }

        return true;
    }
    else
    {
        return false;
    }
}

bool GazeboYarpControlBoardDriver::findMotorControlGroup(yarp::os::Bottle& motorControlGroup_bot) const
{
    if (!m_pluginParameters.check("WRAPPER"))
    {
        yError()<<"Missing WRAPPER group";
        return false;
    }

    if (!m_pluginParameters.findGroup("WRAPPER").check("networks"))
    {
        yError()<<"Missing networks group";
        return false;
    }

    yarp::os::Bottle& name_bot = m_pluginParameters.findGroup("WRAPPER").findGroup("networks");
    std::string name = name_bot.get(1).toString();

    motorControlGroup_bot = m_pluginParameters.findGroup(name);
    return true;
}

bool GazeboYarpControlBoardDriver::setPositionsToleranceLinear()
{
    yarp::os::Bottle kin_chain_bot;
    if (!findMotorControlGroup(kin_chain_bot))
        return false;

    if (!kin_chain_bot.check("positionToleranceLinear")) {
        yWarning()<<"No positionToleranceLinear value found in ini file, default one will be used!";
        return true;
    }

    yarp::os::Bottle& positionToleranceLinear_bot = kin_chain_bot.findGroup("positionToleranceLinear");
    if (static_cast<size_t>(positionToleranceLinear_bot.size()) != 2) {
        yError()<<"Invalid number of params:positionToleranceLinear:"<<static_cast<size_t>(positionToleranceLinear_bot.size());
        return false;
    }

    yarp::os::Value tmp=positionToleranceLinear_bot.get(1);
    if (!tmp.isFloat64())
    {
        yError()<<"Invalid param type:positionToleranceLinear";
        return false;
    }

    m_robotPositionToleranceLinear=tmp.asFloat64();

    yDebug()<<"positionToleranceLinear: [ "<<m_robotPositionToleranceLinear<<" ]";

    return true;
}

bool GazeboYarpControlBoardDriver::setPositionsToleranceRevolute()
{
    yarp::os::Bottle kin_chain_bot;
    if (!findMotorControlGroup(kin_chain_bot))
        return false;

    if (!kin_chain_bot.check("positionToleranceRevolute")) {
        yWarning()<<"No positionToleranceRevolute value found in ini file, default one will be used!";
        return true;
    }

    yarp::os::Bottle& positionToleranceRevolute_bot = kin_chain_bot.findGroup("positionToleranceRevolute");
    if (static_cast<size_t>(positionToleranceRevolute_bot.size()) != 2) {
        yError()<<"Invalid number of params:positionToleranceRevolute:"<<static_cast<size_t>(positionToleranceRevolute_bot.size());
        return false;
    }

    yarp::os::Value tmp=positionToleranceRevolute_bot.get(1);
    if (!tmp.isFloat64())
    {
        yError()<<"Invalid param type:positionToleranceRevolute";
        return false;
    }

    m_robotPositionToleranceRevolute=tmp.asFloat64();

    yDebug()<<"positionToleranceRevolute: [ "<<m_robotPositionToleranceRevolute<<" ]";
    return true;
}

bool GazeboYarpControlBoardDriver::setMaxTorques()
{
    yarp::os::Bottle kin_chain_bot;
    if (!findMotorControlGroup(kin_chain_bot))
        return false;

    if (kin_chain_bot.check("max_torques")) {
        yInfo()<<"max_torques param found!";
        yarp::os::Bottle& max_torque_bot = kin_chain_bot.findGroup("max_torques");
        if(static_cast<size_t>(max_torque_bot.size()) - 1 == m_numberOfJoints) {
            for(size_t i = 0; i < m_numberOfJoints; ++i)
                m_maxTorques[i] = max_torque_bot.get(i+1).asFloat64();
        } else
            yError()<<"Invalid number of params";
    } else
        yWarning()<<"No max torques value found in ini file, default one will be used!";

    yDebug()<<"max_torques: [ "<<m_maxTorques.toString()<<" ]";

    return true;
}

bool GazeboYarpControlBoardDriver::setMinMaxImpedance()
{
    yarp::os::Bottle kin_chain_bot;
    if (!findMotorControlGroup(kin_chain_bot))
        return false;

    if (kin_chain_bot.check("min_stiffness")) {
        yInfo()<<"min_stiffness param found!";
        yarp::os::Bottle& min_stiff_bot = kin_chain_bot.findGroup("min_stiffness");
        if(static_cast<size_t>(min_stiff_bot.size()) - 1 == m_numberOfJoints) {
            for(size_t i = 0; i < m_numberOfJoints; ++i)
                m_minStiffness[i] = min_stiff_bot.get(i+1).asFloat64();
        } else
            yError()<<"Invalid number of params";
    } else
        yWarning()<<"No minimum stiffness value found in ini file, default one will be used!";

    if (kin_chain_bot.check("max_stiffness")) {
        yInfo()<<"max_stiffness param found!";
        yarp::os::Bottle& max_stiff_bot = kin_chain_bot.findGroup("max_stiffness");
        if (static_cast<size_t>(max_stiff_bot.size())-1 == m_numberOfJoints) {
            for (size_t i = 0; i < m_numberOfJoints; ++i)
                m_maxStiffness[i] = max_stiff_bot.get(i+1).asFloat64();
        } else
            yError()<<"Invalid number of params";
    }
    else
        yWarning()<<"No maximum stiffness value found in ini file, default one will be used!";

    if (kin_chain_bot.check("min_damping")) {
        yInfo()<<"min_damping param found!";
        yarp::os::Bottle& min_damping_bot = kin_chain_bot.findGroup("min_damping");
        if(static_cast<size_t>(min_damping_bot.size())-1 == m_numberOfJoints) {
            for(size_t i = 0; i < m_numberOfJoints; ++i)
                m_minDamping[i] = min_damping_bot.get(i+1).asFloat64();
        } else
            yError()<<"Invalid number of params";
    } else
        yWarning()<<"No minimum dampings value found in ini file, default one will be used!";

    if(kin_chain_bot.check("max_damping")) {
        yInfo()<<"max_damping param found!";
        yarp::os::Bottle& max_damping_bot = kin_chain_bot.findGroup("max_damping");
        if (static_cast<size_t>(max_damping_bot.size()) - 1 == m_numberOfJoints) {
            for(size_t i = 0; i < m_numberOfJoints; ++i)
                m_maxDamping[i] = max_damping_bot.get(i+1).asFloat64();
        } else
            yError()<<"Invalid number of params";
    } else
        yWarning()<<"No maximum damping value found in ini file, default one will be used!";

    yDebug()<<"min_stiffness: [ "<<m_minStiffness.toString()<<" ]";
    yDebug()<<"max_stiffness: [ "<<m_maxStiffness.toString()<<" ]";
    yDebug()<<"min_damping: [ "<<m_minDamping.toString()<<" ]";
    yDebug()<<"max_damping: [ "<<m_maxDamping.toString()<<" ]";
    return true;
}

bool GazeboYarpControlBoardDriver::setPIDs()
{
    //POSITION PARAMETERS
    PIDMap::mapped_type& positionPIDs = m_pids[VOCAB_PIDTYPE_POSITION];
    if (m_pluginParameters.check("POSITION_CONTROL"))
    {
        if (!setPIDsForGroup_POSITION(m_position_control_law, positionPIDs))
        {
            yError() << "Error in one parameter of POSITION_CONTROL section";
            return false;
        }
    }
    else if (m_pluginParameters.check("GAZEBO_PIDS"))
    {
        yWarning ("'POSITION_CONTROL' group not found. Using DEPRECATED GAZEBO_PIDS section");
        setPIDsForGroup("GAZEBO_PIDS", positionPIDs, PIDFeedbackTermAllTerms);
        for (size_t i = 0; i < m_numberOfJoints; ++i) {m_position_control_law[i] = "joint_pid_gazebo_v1";}
    }
    else
    {
        yWarning() << "Unable to find a valid section containing position control gains, use default values";
        setPIDsForGroup("GAZEBO_PIDS", positionPIDs, PIDFeedbackTermAllTerms);
        for (size_t i = 0; i < m_numberOfJoints; ++i) {m_position_control_law[i] = "joint_pid_gazebo_v1";}
    }

    //VELOCITY PARAMETERS
    PIDMap::mapped_type& velocityPIDs = m_pids[VOCAB_PIDTYPE_VELOCITY];
    if (m_pluginParameters.check("VELOCITY_CONTROL"))
    {
        if (!setPIDsForGroup_VELOCITY(m_velocity_control_law, velocityPIDs))
        {
            yError() << "Error in one parameter of VELOCITY_CONTROL section";
            return false;
        }
    }
    else if (m_pluginParameters.check("GAZEBO_VELOCITY_PIDS"))
    {
        yWarning ("'VELOCITY_CONTROL' group not found. Using DEPRECATED GAZEBO_VELOCITY_PIDS section");
        setPIDsForGroup("GAZEBO_VELOCITY_PIDS", velocityPIDs, PIDFeedbackTerm(PIDFeedbackTermProportionalTerm | PIDFeedbackTermIntegrativeTerm));
        for (size_t i = 0; i < m_numberOfJoints; ++i) {m_velocity_control_law[i] = "joint_pid_gazebo_v1";}
    }
    else
    {
        yWarning() << "Unable to find a valid section containing velocity control gains, use default values";
        setPIDsForGroup("GAZEBO_VELOCITY_PIDS", velocityPIDs, PIDFeedbackTerm(PIDFeedbackTermProportionalTerm | PIDFeedbackTermIntegrativeTerm));
        for (size_t i = 0; i < m_numberOfJoints; ++i) {m_velocity_control_law[i] = "joint_pid_gazebo_v1";}
    }

    //IMPEDANCE PARAMETERS
    if (m_pluginParameters.check("IMPEDANCE_CONTROL"))
    {
        if (!setPIDsForGroup_IMPEDANCE(m_impedance_control_law, m_impedancePosPDs))
        {
            yError() << "Error in one parameter of IMPEDANCE section";
            return false;
        }
    }
    else if (m_pluginParameters.check("GAZEBO_IMPEDANCE_POSITION_PIDS"))
    {
        yWarning ("'IMPEDANCE' group not found. Using DEPRECATED GAZEBO_PIDS section");
        setPIDsForGroup("GAZEBO_IMPEDANCE_POSITION_PIDS", m_impedancePosPDs, PIDFeedbackTerm(PIDFeedbackTermProportionalTerm | PIDFeedbackTermDerivativeTerm));
        for (size_t i = 0; i < m_numberOfJoints; ++i) {m_impedance_control_law[i] = "joint_pid_gazebo_v1";}
    }
    else
    {
        yWarning() << "Unable to find a valid section containing impedance control gains, use default values";
        setPIDsForGroup("GAZEBO_IMPEDANCE_POSITION_PIDS", m_impedancePosPDs, PIDFeedbackTerm(PIDFeedbackTermProportionalTerm | PIDFeedbackTermDerivativeTerm));
        for (size_t i = 0; i < m_numberOfJoints; ++i) {m_impedance_control_law[i] = "joint_pid_gazebo_v1";}
    }

    if (m_pluginParameters.check("SIMULATION"))
    {
        Bottle& simGroup = m_pluginParameters.findGroup("SIMULATION");
        Bottle xtmp;
        if (!validate(simGroup, xtmp, "kPWM", "kPWM parameter", m_numberOfJoints+1))  {
            yError() << "Missing kPWM parameter";
            return false;
        } else {
            for (size_t j=0; j<m_numberOfJoints; j++) {
                m_kPWM[j]=xtmp.get(j+1).asFloat64();
            }
        }
    }
    return true;
}

bool GazeboYarpControlBoardDriver::check_joint_within_limits_override_torque(int i, double& ref)
{
    const gazebo::common::PID& positionPID = m_pids[VOCAB_PIDTYPE_POSITION][i];

    int   signKp = (positionPID.GetPGain()>0?1:-1);
    int   signRef = (ref>0?1:-1);
    if (m_controlMode[i] == VOCAB_CM_TORQUE || m_interactionMode[i] == VOCAB_IM_COMPLIANT
        || m_controlMode[i] == VOCAB_CM_PWM || m_controlMode[i] == VOCAB_CM_CURRENT )
    {
        if (m_motPositions[i] > m_jointPosLimits[i].max)
        {
            if (signKp*signRef >0 )
            {
                ref = ( (m_jointPosLimits[i].max-m_motPositions[i]) * (positionPID.GetPGain()));
                if (ref > positionPID.GetCmdMax()) ref = positionPID.GetCmdMax();
                else if (ref < positionPID.GetCmdMin()) ref = positionPID.GetCmdMin();
                //_integral[i] = 0;
#ifdef DEBUG_LIMITS
                yDebug() << "TTT TMAX" << m_motPositions[i] <<">" <<  m_jointPosLimits[i].max;
#endif
            }
            return false;
        }
        else if (m_motPositions[i] < m_jointPosLimits[i].min)
        {
            if (signKp*signRef <0 )
            {
                ref = ( (m_jointPosLimits[i].min-m_motPositions[i]) * (positionPID.GetPGain()));
                if (ref > positionPID.GetCmdMax()) ref = positionPID.GetCmdMax();
                else if (ref < positionPID.GetCmdMin()) ref = positionPID.GetCmdMin();
                //_integral[i] = 0;
#ifdef DEBUG_LIMITS
                yDebug() << "TTT TMIN" << m_motPositions[i] <<"<" <<  m_jointPosLimits[i].min;
#endif
            }
            return false;
        }
    }
    return true;
}

double GazeboYarpControlBoardDriver::convertGazeboToUser(int joint, double value)
{
    double newValue = 0;
    switch(m_jointTypes[joint])
    {
        case JointType_Revolute:
        {
            newValue = GazeboYarpPlugins::convertRadiansToDegrees(value);
            break;
        }

        case JointType_Prismatic:
        {
            // For prismatic joints internal representation is already meter, nothing to do here.
            newValue = value;
            break;
        }

        case JointType_Unknown:
        {
            yError() << "Cannot convert measure from Gazebo to User units, type of joint not supported for axes " <<
            m_jointNames[joint] << " type is " << m_jointTypes[joint];
            break;
        }
    }
    return newValue;
}

double * GazeboYarpControlBoardDriver::convertGazeboToUser(double *values)
{
    for(size_t i = 0; i < m_numberOfJoints; i++)
        values[i] = convertGazeboToUser(i, values[i]);
    return values;
}

double GazeboYarpControlBoardDriver::convertUserToGazebo(int joint, double value)
{
    double newValue = 0;
    switch(m_jointTypes[joint])
    {
        case JointType_Revolute:
        {
            newValue = GazeboYarpPlugins::convertDegreesToRadians(value);
            break;
        }

        case JointType_Prismatic:
        {
            newValue = value;
            break;
        }

        case JointType_Unknown:
        {
            yError() << "Cannot convert measure from User to Gazebo units, type of joint not supported";
            break;
        }
    }
    return newValue;
}

double * GazeboYarpControlBoardDriver::convertUserToGazebo(double *values)
{
    for (size_t i = 0; i < m_numberOfJoints; i++) {
        values[i] = convertGazeboToUser(i, values[i]);
    }
    return values;
}

double GazeboYarpControlBoardDriver::convertUserGainToGazeboGain(int joint, double value)
{
    double newValue = 0;
    switch(m_jointTypes[joint])
    {
        case JointType_Revolute:
        {
            newValue = GazeboYarpPlugins::convertRadiansToDegrees(value);
            break;
        }

        case JointType_Prismatic:
        {
            newValue = value;
            break;
        }

        case JointType_Unknown:
        {
            yError() << "Cannot convert measure from User to Gazebo units, type of joint not supported";
            break;
        }
    }
    return newValue;
}

double GazeboYarpControlBoardDriver::convertGazeboGainToUserGain(int joint, double value)
{
    double newValue = 0;
    switch(m_jointTypes[joint])
    {
        case JointType_Revolute:
        {
            newValue = GazeboYarpPlugins::convertDegreesToRadians(value);
            break;
        }

        case JointType_Prismatic:
        {
            newValue = value;
            break;
        }

        case JointType_Unknown:
        {
            yError() << "Cannot convert measure from Gazebo gains to User gain units, type of joint not supported for axes " <<
            m_jointNames[joint] << " type is " << m_jointTypes[joint];
            break;
        }
    }
    return newValue;
}
