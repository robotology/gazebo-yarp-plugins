/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "ControlBoardDriver.h"
#include <GazeboYarpPlugins/common.h>

#include <cstdio>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/math/Angle.hh>

#include <yarp/os/LogStream.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;


const double RobotPositionTolerance_revolute = 0.9;      // Degrees
const double RobotPositionTolerance_linear   = 0.004;    // Meters

GazeboYarpControlBoardDriver::GazeboYarpControlBoardDriver() : deviceName("") {}

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
    m_zeroPosition.resize(m_numberOfJoints);
    m_jntReferenceVelocities.resize(m_numberOfJoints);
    m_velocities.resize(m_numberOfJoints);
    amp.resize(m_numberOfJoints);
    m_torques.resize(m_numberOfJoints); m_torques.zero();
    m_maxTorques.resize(m_numberOfJoints, 2000.0);
    m_trajectoryGenerationReferenceSpeed.resize(m_numberOfJoints);
    m_jntReferencePositions.resize(m_numberOfJoints);
    m_oldReferencePositions.resize(m_numberOfJoints);
    m_trajectoryGenerationReferencePosition.resize(m_numberOfJoints);
    m_trajectoryGenerationReferenceAcceleration.resize(m_numberOfJoints);
    m_jntReferenceTorques.resize(m_numberOfJoints);
    m_jointPosLimits.resize(m_numberOfJoints);
    m_jointVelLimits.resize(m_numberOfJoints);
    m_positionPIDs.reserve(m_numberOfJoints);
    m_velocityPIDs.reserve(m_numberOfJoints);
    m_impedancePosPDs.reserve(m_numberOfJoints);
    m_torquePIDs.resize(m_numberOfJoints);
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

    // Initial zeroing of all vectors
    m_positions.zero();
    m_zeroPosition.zero();
    m_velocities.zero();
    m_trajectoryGenerationReferenceSpeed.zero();
    m_jntReferencePositions.zero();
    m_jntReferenceVelocities.zero();
    m_jntReferenceTorques.zero();
    m_trajectoryGenerationReferencePosition.zero();
   // m_trajectoryGenerationReferenceAcceleration.zero();
    m_trajectoryGenerationReferenceAcceleration=10.0; //default value in deg/s^2
    amp = 1; // initially on - ok for simulator
    m_controlMode = new int[m_numberOfJoints];
    m_interactionMode = new int[m_numberOfJoints];
    m_isMotionDone = new bool[m_numberOfJoints];
    m_clock = 0;
    m_torqueOffset = 0;

    m_trajectory_generator.resize(m_numberOfJoints, NULL);
    m_coupling_handler.clear();
    m_speed_ramp_handler.resize(m_numberOfJoints, NULL);
    VectorOf<int> trajectory_generator_type;
    trajectory_generator_type.resize(m_numberOfJoints);
    
    yarp::os::Bottle& traj_bottle = m_pluginParameters.findGroup("TRAJECTORY_GENERATION");
    if (!traj_bottle.isNull())
    {
       yarp::os::Bottle& traj_type = traj_bottle.findGroup("trajectory_type");
       if (!traj_type.isNull())
       {
           if       (traj_type.get(1).asString()=="minimum_jerk")   {for(unsigned int i = 0; i < m_numberOfJoints; ++i) {trajectory_generator_type[i]=yarp::dev::TRAJECTORY_TYPE_MIN_JERK;}}
           else if  (traj_type.get(1).asString()=="constant_speed") {for(unsigned int i = 0; i < m_numberOfJoints; ++i) {trajectory_generator_type[i]=yarp::dev::TRAJECTORY_TYPE_CONST_SPEED;}}
           else                                                     {for(unsigned int i = 0; i < m_numberOfJoints; ++i) {trajectory_generator_type[i]=yarp::dev::TRAJECTORY_TYPE_MIN_JERK;}}
       }
       else 
       {
         yWarning() << "Missing TRAJECTORY_GENERATION group. Missing trajectory_type param. Assuming minimum_jerk";
         {for(unsigned int i = 0; i < m_numberOfJoints; ++i) {trajectory_generator_type[i]=yarp::dev::TRAJECTORY_TYPE_MIN_JERK;}}
       }
    }
    else
    {
       yWarning() << "Missing trajectory_type param. Assuming minimum_jerk";
       {for(unsigned int i = 0; i < m_numberOfJoints; ++i) {trajectory_generator_type[i]=yarp::dev::TRAJECTORY_TYPE_MIN_JERK;}}
    }
       
    for (unsigned int j = 0; j < m_numberOfJoints; ++j)
    {
      if (trajectory_generator_type[j]==yarp::dev::TRAJECTORY_TYPE_MIN_JERK)
         {m_trajectory_generator[j] = new MinJerkTrajectoryGenerator(m_robot);}
      else
         {m_trajectory_generator[j] = new ConstSpeedTrajectoryGenerator(m_robot);}
    }
    
    for (unsigned int j = 0; j < m_numberOfJoints; ++j)
    {
      m_speed_ramp_handler[j] = new RampFilter();
    }
    
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
       
       for (size_t cnt=1; cnt<coupling_group_bottle.size() ;cnt++)
       { 
         yarp::os::Bottle* coupling_bottle = coupling_group_bottle.get(cnt).asList();
         
         if (coupling_bottle==0 || coupling_bottle->size()!=3)
         {
           yError() << "Error parsing coupling parameter"; return false;
         } 
         //yDebug() << "Requested coupling:" << cnt << " / " << coupling_bottle->size();
         //yDebug() << "Requested coupling:" << coupling_bottle->toString();
         
         yarp::sig::VectorOf<int> coupled_joints;
         std::vector<std::string> coupled_joint_names;
         Bottle* b = coupling_bottle->get(1).asList();
         if (b==0 || b->size()==0) {yError() << "Error parsing coupling parameter, wrong size of the joints numbers list"; return false;}
         for (int is=0;is<b->size();is++) { coupled_joints.push_back(b->get(is).asInt()); }
         Bottle* b2 = coupling_bottle->get(2).asList();
         if (b2==0 || b2->size()==0) {yError() << "Error parsing coupling parameter, wrong size of the joint names list"; return false;}
         for (int is=0;is<b2->size();is++) { coupled_joint_names.push_back(b2->get(is).asString()); }
             
         if (coupling_bottle->get(0).asString()=="eyes_vergence_control")
         {
             BaseCouplingHandler* cpl = new EyesCouplingHandler(m_robot,coupled_joints, coupled_joint_names);
             m_coupling_handler.push_back(cpl);
             yInfo() << "using eyes_vergence_control";
         }
         else if (coupling_bottle->get(0).asString()=="fingers_abduction_control")
         {
             BaseCouplingHandler* cpl = new FingersAbductionCouplingHandler(m_robot,coupled_joints, coupled_joint_names);
             m_coupling_handler.push_back(cpl);
             yInfo() << "using fingers_abduction_control";
         }
         else if (coupling_bottle->get(0).asString()=="thumb_control")
         {
             BaseCouplingHandler* cpl = new ThumbCouplingHandler(m_robot,coupled_joints, coupled_joint_names);
             m_coupling_handler.push_back(cpl);
             yInfo() << "using thumb_control";
         }
         else if (coupling_bottle->get(0).asString()=="index_control")
         {
             BaseCouplingHandler* cpl = new IndexCouplingHandler(m_robot,coupled_joints, coupled_joint_names);
             m_coupling_handler.push_back(cpl);
             yInfo() << "using index_control";
         }
         else if (coupling_bottle->get(0).asString()=="middle_control")
         {
             BaseCouplingHandler* cpl = new MiddleCouplingHandler(m_robot,coupled_joints, coupled_joint_names);
             m_coupling_handler.push_back(cpl);
             yInfo() << "using middle_control";
         }
         else if (coupling_bottle->get(0).asString()=="pinky_control")
         {
             BaseCouplingHandler* cpl = new PinkyCouplingHandler(m_robot,coupled_joints, coupled_joint_names);
             m_coupling_handler.push_back(cpl);
             yInfo() << "using pinky_control";
         }
         else if (coupling_bottle->get(0).asString()=="cer_hand")
         {
             BaseCouplingHandler* cpl = new CerHandCouplingHandler(m_robot,coupled_joints, coupled_joint_names);
             m_coupling_handler.push_back(cpl);
             yInfo() << "using cer_hand_control";
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
    yDebug() << "done";
    for (unsigned int j = 0; j < m_numberOfJoints; ++j)
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


    if (!setMinMaxVel())
    {
      yError()<<"Failed to get Velocity Limits";
      //return false; //to be added soon
    }
    for (unsigned int j = 0; j < m_numberOfJoints; ++j)
    {
        // NOTE: This has to be after setMinMaxVel function
        m_trajectoryGenerationReferenceSpeed[j]=0.0;
        if      (m_jointTypes[j]==JointType_Revolute)  m_trajectoryGenerationReferenceSpeed[j] = 10.0; //deg/s
        else if (m_jointTypes[j]==JointType_Prismatic) m_trajectoryGenerationReferenceSpeed[j] = 0.010; //m/s
        if (m_trajectoryGenerationReferenceSpeed[j] >  m_jointVelLimits[j].max) m_trajectoryGenerationReferenceSpeed[j]= m_jointVelLimits[j].max/10;
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

    m_gazeboNode = gazebo::transport::NodePtr(new gazebo::transport::Node);
    m_gazeboNode->Init(this->m_robot->GetWorld()->GetName());
    m_jointCommandPublisher = m_gazeboNode->Advertise<gazebo::msgs::JointCmd>(std::string("~/") + this->m_robot->GetName() + "/joint_cmd");

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
#if GAZEBO_MAJOR_VERSION >= 4
            m_jointPointers[i]->SetPosition(0,initial_config[i]);
#else
            gazebo::math::Angle a;
            a.SetFromRadian(initial_config[i]);
            m_jointPointers[i]->SetAngle(0,a);
#endif
        }

        yDebug() << "Initializing Trajectory Generator with default values";
        for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
            m_trajectory_generator[i]->setLimits(m_jointPosLimits[i].min,m_jointPosLimits[i].max);
            m_trajectory_generator[i]->initTrajectory(m_positions[i],m_positions[i],m_trajectoryGenerationReferenceSpeed[i]);
        }
    }
    else
    {
        yDebug() << "Initializing Trajectory Generator with current values";
        for (unsigned int i = 0; i < m_numberOfJoints; ++i)
        {
            m_positions[i] = convertGazeboToUser(i, m_jointPointers[i]->GetAngle(0));
            m_trajectory_generator[i]->setLimits(m_jointPosLimits[i].min,m_jointPosLimits[i].max);
            m_trajectory_generator[i]->initTrajectory(m_positions[i],m_positions[i],m_trajectoryGenerationReferenceSpeed[i]);
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
    for(int i=0; i< m_numberOfJoints; i++)
    {
        switch( m_jointPointers[i]->GetType())
        {
            case ( gazebo::physics::Entity::HINGE_JOINT  |  gazebo::physics::Entity::JOINT):
            {
                m_jointTypes[i] = JointType_Revolute;
                m_positionThreshold[i] = RobotPositionTolerance_revolute;
                break;
            }

            case ( gazebo::physics::Entity::SLIDER_JOINT |  gazebo::physics::Entity::JOINT):
            {
                m_jointTypes[i] = JointType_Prismatic;
                m_positionThreshold[i] = RobotPositionTolerance_linear;
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
    m_clock++;
    //yDebug()<< deviceName << "1";
    // measurements acquisition
    for (unsigned int jnt_cnt = 0; jnt_cnt < m_jointPointers.size(); jnt_cnt++)
    {
        //TODO: consider multi-dof joint ?
        m_positions[jnt_cnt] = convertGazeboToUser(jnt_cnt, m_jointPointers[jnt_cnt]->GetAngle(0));
        m_velocities[jnt_cnt] = convertGazeboToUser(jnt_cnt, m_jointPointers[jnt_cnt]->GetVelocity(0));
        m_torques[jnt_cnt] = m_jointPointers[jnt_cnt]->GetForce(0u);
    }
    
    //yDebug()<< deviceName << "2";
    //measurements decoupling
    for (unsigned int cpl_cnt = 0; cpl_cnt < m_coupling_handler.size(); cpl_cnt++)
    {
      if (m_coupling_handler[cpl_cnt])
      {
         m_coupling_handler[cpl_cnt]->decouplePos(m_positions);
         m_coupling_handler[cpl_cnt]->decoupleVel(m_velocities);
         //m_coupling_handler[cpl_cnt]->decoupleAcc(m_accelerations); //missing
         m_coupling_handler[cpl_cnt]->decoupleTrq(m_torques);
      } 
    }
    
    //yDebug()<< deviceName << "3";
    // check measured torque for hw fault
    for (unsigned int jnt_cnt = 0; jnt_cnt < m_jointPointers.size(); jnt_cnt++) {
      if (m_controlMode[jnt_cnt]!=VOCAB_CM_HW_FAULT && fabs(m_torques[jnt_cnt])>m_maxTorques[jnt_cnt])
      {
        m_controlMode[jnt_cnt]=VOCAB_CM_HW_FAULT;
        yError() << "An hardware fault occurred on joint "<< jnt_cnt << " torque too big! ( " << m_torques[jnt_cnt] << " )";
      }
    }

    // Updating timestamp
    m_lastTimestamp.update(_info.simTime.Double());

    //logger.log(m_velocities[2]);
 
    //update refernce m_jntReferenceVelocities
    for (unsigned int j = 0; j < m_numberOfJoints; ++j)
    {
      if (m_speed_ramp_handler[j])
      {
        m_speed_ramp_handler[j]->update();
        //yDebug() << m_speed_ramp_handler[j]->getCurrentValue();
      }
    }
    
    //yDebug()<< deviceName << "4";      
    //update Trajectories
    for (unsigned int j = 0; j < m_numberOfJoints; ++j)
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
                double computed_ref_speed = m_speed_ramp_handler[j]->getCurrentValue() / 1000.0 * 1;  //controller period
                double computed_ref_pos =  m_jntReferencePositions[j] + m_trajectory_generator[j]->computeTrajectoryStep();
                m_jntReferencePositions[j] = computed_ref_pos + computed_ref_speed;
                //yDebug() << computed_ref_pos << " " << computed_ref_speed;
                m_isMotionDone[j] = m_trajectory_generator[j]->isMotionDone();
            }        
        }
    }
    
    //yDebug()<< deviceName << "5";     
    Vector m_motReferencePositions=m_jntReferencePositions;
    Vector m_motReferenceVelocities=m_jntReferenceVelocities;
    Vector m_motReferenceTorques=m_jntReferenceTorques;
         
    //references decoupling
    for (unsigned int cpl_cnt = 0; cpl_cnt < m_coupling_handler.size(); cpl_cnt++)
    {
      if (m_coupling_handler[cpl_cnt])
      {
        m_motReferencePositions = m_coupling_handler[cpl_cnt]->decoupleRefPos(m_jntReferencePositions);
        m_motReferenceVelocities = m_coupling_handler[cpl_cnt]->decoupleRefVel(m_jntReferenceVelocities);
        m_motReferenceTorques = m_coupling_handler[cpl_cnt]->decoupleRefTrq(m_jntReferenceTorques);
      }
    }
    
    //yDebug()<< deviceName << "6";       
    //update References
    for (unsigned int j = 0; j < m_numberOfJoints; ++j) {
        //set pos joint value, set m_referenceVelocities joint value
        if ((m_controlMode[j] == VOCAB_CM_POSITION || m_controlMode[j] == VOCAB_CM_POSITION_DIRECT) && (m_interactionMode[j] == VOCAB_IM_STIFF)) {
            if (m_clock % _T_controller == 0) {
                sendPositionToGazebo(j, m_motReferencePositions[j]);
            }
        } else if ((m_controlMode[j] == VOCAB_CM_POSITION || m_controlMode[j] == VOCAB_CM_POSITION_DIRECT) && (m_interactionMode[j] == VOCAB_IM_COMPLIANT)) {
            if (m_clock % _T_controller == 0) {
                sendImpPositionToGazebo(j, m_motReferencePositions[j]);
            }
        } else if ((m_controlMode[j] == VOCAB_CM_VELOCITY) && (m_interactionMode[j] == VOCAB_IM_STIFF)) {//set vmo joint value
            if (m_clock % _T_controller == 0) {
                if (m_speed_ramp_handler[j]) {sendVelocityToGazebo(j, m_speed_ramp_handler[j]->getCurrentValue());}
                else {sendVelocityToGazebo(j, m_jntReferenceVelocities[j]);}
            }
        } else if ((m_controlMode[j] == VOCAB_CM_VELOCITY) && (m_interactionMode[j] == VOCAB_IM_COMPLIANT)) {
            if (m_clock % _T_controller == 0) {
                yWarning("Compliant velocity control not yet implemented");
                if (m_speed_ramp_handler[j]) {sendVelocityToGazebo(j, m_speed_ramp_handler[j]->getCurrentValue());}
                else {sendVelocityToGazebo(j, m_jntReferenceVelocities[j]);}
            }
        } else if ((m_controlMode[j] == VOCAB_CM_MIXED) && (m_interactionMode[j] == VOCAB_IM_STIFF)) {
            if (m_clock % _T_controller == 0) {
                sendPositionToGazebo(j, m_motReferencePositions[j]);
            }
        } else if ((m_controlMode[j] == VOCAB_CM_MIXED) && (m_interactionMode[j] == VOCAB_IM_COMPLIANT)) {
            if (m_clock % _T_controller == 0) {
                sendImpPositionToGazebo(j, m_motReferencePositions[j]);
            }
        } else if (m_controlMode[j] == VOCAB_CM_TORQUE) {
            if (m_clock % _T_controller == 0) {
                sendTorqueToGazebo(j, m_motReferenceTorques[j]);
            }
        } else if (m_controlMode[j] == VOCAB_CM_IDLE) {
            if (m_clock % _T_controller == 0) {
                sendTorqueToGazebo(j, 0.0);
            }
        } else if (m_controlMode[j] == VOCAB_CM_HW_FAULT) {
            if (m_clock % _T_controller == 0) {
                sendTorqueToGazebo(j, 0.0);
            }
        } else if (m_controlMode[j] == VOCAB_CM_OPENLOOP) {
            //OpenLoop control sends torques to gazebo at this moment.
            //Check if gazebo implements a "motor" entity and change the code accordingly.
            if (m_clock % _T_controller == 0) {
                sendTorqueToGazebo(j, m_motReferenceTorques[j]);
            }
        }
    }
    //yDebug()<< deviceName << "7";  
}

void GazeboYarpControlBoardDriver::onReset()
{
    resetPositionsAndTrajectoryGenerators();
}


bool GazeboYarpControlBoardDriver::setMinMaxPos()
{
    for(unsigned int i = 0; i < m_numberOfJoints; ++i)
    {
        m_jointPosLimits[i].max = convertGazeboToUser(i, m_jointPointers[i]->GetUpperLimit(0));
        m_jointPosLimits[i].min = convertGazeboToUser(i, m_jointPointers[i]->GetLowerLimit(0));
    }
    
    //...if the yarp plugin configuration file specifies a different velocity, override it...
    yarp::os::Bottle& limits_bottle = m_pluginParameters.findGroup("LIMITS");
    if (!limits_bottle.isNull())
    {
       yarp::os::Bottle& pos_limit_max = limits_bottle.findGroup("jntPosMax");
       if (!pos_limit_max.isNull() && pos_limit_max.size() == m_numberOfJoints+1)
       {
           for(unsigned int i = 0; i < m_numberOfJoints; ++i)
           { 
             m_jointPosLimits[i].max = pos_limit_max.get(i+1).asDouble();
         
           }
       }
       else
       {
           yError() << "Failed to parse jntPosMax parameter";
           return false;
       }
       yarp::os::Bottle& pos_limit_min = limits_bottle.findGroup("jntPosMin");
       if (!pos_limit_min.isNull() && pos_limit_min.size() == m_numberOfJoints+1)
       {
           for(unsigned int i = 0; i < m_numberOfJoints; ++i)
           { 
             m_jointPosLimits[i].min = pos_limit_min.get(i+1).asDouble();           
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
    for(unsigned int i = 0; i < m_numberOfJoints; ++i)
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
           for(unsigned int i = 0; i < m_numberOfJoints; ++i)
           { 
             m_jointVelLimits[i].max = vel_limits.get(i+1).asDouble();
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
    for (unsigned int i = 0; i < m_jointNames.size(); i++) {
        bool joint_found = false;
        controlboard_joint_names.push_back(joint_names_bottle.get(i+1).asString().c_str());

        for (unsigned int gazebo_joint = 0; gazebo_joint < gazebo_models_joints.size() && !joint_found; gazebo_joint++) {
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
                                                   std::vector<GazeboYarpControlBoardDriver::PID>& pids,
                                                   enum PIDFeedbackTerm pidTerms)
{
    yarp::os::Property prop;
    if (m_pluginParameters.check(pidGroupName.c_str())) {
        for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
            std::stringstream property_name;
            property_name<<"Pid";
            property_name<<i;

            yarp::os::Bottle& pid = m_pluginParameters.findGroup(pidGroupName.c_str()).findGroup(property_name.str().c_str());

            GazeboYarpControlBoardDriver::PID pidValue = {0, 0, 0, -1, -1};
            if (pidTerms & PIDFeedbackTermProportionalTerm)
                pidValue.p = pid.get(1).asDouble();
            if (pidTerms & PIDFeedbackTermDerivativeTerm)
                pidValue.d = pid.get(2).asDouble();
            if (pidTerms & PIDFeedbackTermIntegrativeTerm)
                pidValue.i = pid.get(3).asDouble();

            pidValue.maxInt = pid.get(4).asDouble();
            pidValue.maxOut = pid.get(5).asDouble();


            pids.push_back(pidValue);
        }
    } else {
        double default_p = pidTerms & PIDFeedbackTermProportionalTerm ? 500.0 : 0;
        double default_i = pidTerms & PIDFeedbackTermIntegrativeTerm ? 0.1 : 0;
        double default_d = pidTerms & PIDFeedbackTermDerivativeTerm ? 1.0 : 0;
        yWarning()<<"PID gain information not found in group " << pidGroupName << ", using default gains ( "
        <<"P " << default_p << " I " << default_i << " D " << default_d << " )";
        for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
            GazeboYarpControlBoardDriver::PID pid = {500, 0.1, 1.0, -1, -1};
            pids.push_back(pid);
        }
    }

    return true;
}

bool GazeboYarpControlBoardDriver::setPIDsForGroup_POSITION(std::vector<std::string>& control_law, std::vector<GazeboYarpControlBoardDriver::PID>& pids )
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

        yarp::dev::Pid* yarpPid;
        yarpPid = new yarp::dev::Pid[m_numberOfJoints];
        bool error=false;
        unsigned int j=0;

        //control parameters
        if (!validate(pidGroup, xtmp, "kp", "Pid kp parameter", m_numberOfJoints+1))           {error=true;} else {for (j=0; j<m_numberOfJoints; j++) yarpPid[j].kp = xtmp.get(j+1).asDouble();}
        if (!validate(pidGroup, xtmp, "kd", "Pid kd parameter", m_numberOfJoints+1))           {error=true;} else {for (j=0; j<m_numberOfJoints; j++) yarpPid[j].kd = xtmp.get(j+1).asDouble();}
        if (!validate(pidGroup, xtmp, "ki", "Pid kp parameter", m_numberOfJoints+1))           {error=true;} else {for (j=0; j<m_numberOfJoints; j++) yarpPid[j].ki = xtmp.get(j+1).asDouble();}
        if (!validate(pidGroup, xtmp, "maxInt", "Pid maxInt parameter", m_numberOfJoints+1))   {error=true;} else {for (j=0; j<m_numberOfJoints; j++) yarpPid[j].max_int = xtmp.get(j+1).asDouble();}
        if (!validate(pidGroup, xtmp, "maxOutput", "Pid maxOutput parameter", m_numberOfJoints+1))   {error=true;} else {for (j=0; j<m_numberOfJoints; j++) yarpPid[j].max_output = xtmp.get(j+1).asDouble();}
        if (!validate(pidGroup, xtmp, "shift", "Pid shift parameter", m_numberOfJoints+1))     {error=true;} else {for (j=0; j<m_numberOfJoints; j++) yarpPid[j].scale = xtmp.get(j+1).asDouble();}
        if (!validate(pidGroup, xtmp, "ko", "Pid ko parameter", m_numberOfJoints+1))           {error=true;} else {for (j=0; j<m_numberOfJoints; j++) yarpPid[j].offset = xtmp.get(j+1).asDouble();}
        if (!validate(pidGroup, xtmp, "stictionUp", "Pid stictionUp", m_numberOfJoints+1))     {error=true;} else {for (j=0; j<m_numberOfJoints; j++) yarpPid[j].stiction_up_val = xtmp.get(j+1).asDouble();}
        if (!validate(pidGroup, xtmp, "stictionDwn", "Pid stictionDwn", m_numberOfJoints+1))   {error=true;} else {for (j=0; j<m_numberOfJoints; j++) yarpPid[j].stiction_down_val = xtmp.get(j+1).asDouble();}

        if (error)
        {
           delete [] yarpPid;
           return false;
        }
        else
        {
          for (j = 0; j < m_numberOfJoints; ++j)
          {
              if (c_units==metric)
              {
                GazeboYarpControlBoardDriver::PID pidValue;
                pidValue.p=convertUserGainToGazeboGain(j,yarpPid[j].kp)/pow(2,yarpPid[j].scale);
                pidValue.i=convertUserGainToGazeboGain(j,yarpPid[j].ki)/pow(2,yarpPid[j].scale);
                pidValue.d=convertUserGainToGazeboGain(j,yarpPid[j].kd)/pow(2,yarpPid[j].scale);
                pidValue.maxInt=yarpPid[j].max_int;
                pidValue.maxOut=yarpPid[j].max_output;
                pids.push_back(pidValue);
              }
              else if (c_units==si)
              {
                GazeboYarpControlBoardDriver::PID pidValue;
                pidValue.p=yarpPid[j].kp/pow(2,yarpPid[j].scale);
                pidValue.i=yarpPid[j].ki/pow(2,yarpPid[j].scale);
                pidValue.d=yarpPid[j].kd/pow(2,yarpPid[j].scale);
                pidValue.maxInt=yarpPid[j].max_int;
                pidValue.maxOut=yarpPid[j].max_output;
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

bool GazeboYarpControlBoardDriver::setPIDsForGroup_VELOCITY(std::vector<std::string>& control_law, std::vector<GazeboYarpControlBoardDriver::PID>& pids )
{
    yarp::os::Property prop;
    if (m_pluginParameters.check("VELOCITY_CONTROL"))
    {
        Bottle xtmp;
        Bottle pidGroup = m_pluginParameters.findGroup("VELOCITY_CONTROL");

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

        yarp::dev::Pid* yarpPid;
        yarpPid = new yarp::dev::Pid[m_numberOfJoints];
        bool error=false;
        unsigned int j=0;

        //control parameters
        if (!validate(pidGroup, xtmp, "kp", "Pid kp parameter", m_numberOfJoints+1))           {error=true;} else {for (j=0; j<m_numberOfJoints; j++) yarpPid[j].kp = xtmp.get(j+1).asDouble();}
        if (!validate(pidGroup, xtmp, "kd", "Pid kd parameter", m_numberOfJoints+1))           {error=true;} else {for (j=0; j<m_numberOfJoints; j++) yarpPid[j].kd = xtmp.get(j+1).asDouble();}
        if (!validate(pidGroup, xtmp, "ki", "Pid kp parameter", m_numberOfJoints+1))           {error=true;} else {for (j=0; j<m_numberOfJoints; j++) yarpPid[j].ki = xtmp.get(j+1).asDouble();}
        if (!validate(pidGroup, xtmp, "maxInt", "Pid maxInt parameter", m_numberOfJoints+1))   {error=true;} else {for (j=0; j<m_numberOfJoints; j++) yarpPid[j].max_int = xtmp.get(j+1).asDouble();}
        if (!validate(pidGroup, xtmp, "maxOutput", "Pid maxOutput parameter", m_numberOfJoints+1))   {error=true;} else {for (j=0; j<m_numberOfJoints; j++) yarpPid[j].max_output = xtmp.get(j+1).asDouble();}
        if (!validate(pidGroup, xtmp, "shift", "Pid shift parameter", m_numberOfJoints+1))     {error=true;} else {for (j=0; j<m_numberOfJoints; j++) yarpPid[j].scale = xtmp.get(j+1).asDouble();}
        if (!validate(pidGroup, xtmp, "ko", "Pid ko parameter", m_numberOfJoints+1))           {error=true;} else {for (j=0; j<m_numberOfJoints; j++) yarpPid[j].offset = xtmp.get(j+1).asDouble();}

        if (error)
        {
           delete [] yarpPid;
           return false;
        }
        else
        {
          for (j = 0; j < m_numberOfJoints; ++j)
          {
              if (c_units==metric)
              {
                GazeboYarpControlBoardDriver::PID pidValue;
                pidValue.p=convertUserGainToGazeboGain(j,yarpPid[j].kp)/pow(2,yarpPid[j].scale);
                pidValue.i=convertUserGainToGazeboGain(j,yarpPid[j].ki)/pow(2,yarpPid[j].scale);
                pidValue.d=convertUserGainToGazeboGain(j,yarpPid[j].kd)/pow(2,yarpPid[j].scale);
                pidValue.maxInt=yarpPid[j].max_int;
                pidValue.maxOut=yarpPid[j].max_output;
                pids.push_back(pidValue);
              }
              else if (c_units==si)
              {
                GazeboYarpControlBoardDriver::PID pidValue;
                pidValue.p=yarpPid[j].kp/pow(2,yarpPid[j].scale);
                pidValue.i=yarpPid[j].ki/pow(2,yarpPid[j].scale);
                pidValue.d=yarpPid[j].kd/pow(2,yarpPid[j].scale);
                pidValue.maxInt=yarpPid[j].max_int;
                pidValue.maxOut=yarpPid[j].max_output;
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

bool GazeboYarpControlBoardDriver::setPIDsForGroup_IMPEDANCE(std::vector<std::string>& control_law, std::vector<GazeboYarpControlBoardDriver::PID>& pids )
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

        double* stiffness;
        double* damping;
        stiffness = new double[m_numberOfJoints];
        damping   = new double[m_numberOfJoints];
        bool error=false;
        unsigned int j=0;

        //control parameters
        if (!validate(pidGroup, xtmp, "stiffness", "stiffness", m_numberOfJoints+1))    {error=true;} else {for (j=0; j<m_numberOfJoints; j++) stiffness[j] = xtmp.get(j+1).asDouble();}
        if (!validate(pidGroup, xtmp, "damping", "damping", m_numberOfJoints+1))        {error=true;} else {for (j=0; j<m_numberOfJoints; j++) damping[j] = xtmp.get(j+1).asDouble();}

        if (error)
        {
           delete [] stiffness;
           delete [] damping;
           return false;
        }
        else
        {
          for (j = 0; j < m_numberOfJoints; ++j)
          {
              if (c_units==metric)
              {
                GazeboYarpControlBoardDriver::PID pidValue;
                pidValue.p = convertUserGainToGazeboGain(j,stiffness[j]);
                pidValue.i = 0;
                pidValue.d = convertUserGainToGazeboGain(j,damping[j]);
                pidValue.maxInt = 0;
                pidValue.maxOut = 0;
                pids.push_back(pidValue);
              }
              else if (c_units==si)
              {
                GazeboYarpControlBoardDriver::PID pidValue;
                pidValue.p = stiffness[j];
                pidValue.i = 0;
                pidValue.d = damping[j];
                pidValue.maxInt = 0;
                pidValue.maxOut = 0;
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

bool GazeboYarpControlBoardDriver::setMinMaxImpedance()
{

    yarp::os::Bottle& name_bot = m_pluginParameters.findGroup("WRAPPER").findGroup("networks");
    std::string name = name_bot.get(1).toString();

    yarp::os::Bottle& kin_chain_bot = m_pluginParameters.findGroup(name);
    if (kin_chain_bot.check("min_stiffness")) {
        yInfo()<<"min_stiffness param found!";
        yarp::os::Bottle& min_stiff_bot = kin_chain_bot.findGroup("min_stiffness");
        if(min_stiff_bot.size()-1 == m_numberOfJoints) {
            for(unsigned int i = 0; i < m_numberOfJoints; ++i)
                m_minStiffness[i] = min_stiff_bot.get(i+1).asDouble();
        } else
            yError()<<"Invalid number of params";
    } else
        yWarning()<<"No minimum stiffness value found in ini file, default one will be used!";

    if (kin_chain_bot.check("max_stiffness")) {
        yInfo()<<"max_stiffness param found!";
        yarp::os::Bottle& max_stiff_bot = kin_chain_bot.findGroup("max_stiffness");
        if (max_stiff_bot.size()-1 == m_numberOfJoints) {
            for (unsigned int i = 0; i < m_numberOfJoints; ++i)
                m_maxStiffness[i] = max_stiff_bot.get(i+1).asDouble();
        } else
            yError()<<"Invalid number of params";
    }
    else
        yWarning()<<"No maximum stiffness value found in ini file, default one will be used!";

    if (kin_chain_bot.check("min_damping")) {
        yInfo()<<"min_damping param found!";
        yarp::os::Bottle& min_damping_bot = kin_chain_bot.findGroup("min_damping");
        if(min_damping_bot.size()-1 == m_numberOfJoints) {
            for(unsigned int i = 0; i < m_numberOfJoints; ++i)
                m_minDamping[i] = min_damping_bot.get(i+1).asDouble();
        } else
            yError()<<"Invalid number of params";
    } else
        yWarning()<<"No minimum dampings value found in ini file, default one will be used!";

    if(kin_chain_bot.check("max_damping")) {
        yInfo()<<"max_damping param found!";
        yarp::os::Bottle& max_damping_bot = kin_chain_bot.findGroup("max_damping");
        if (max_damping_bot.size() - 1 == m_numberOfJoints) {
            for(unsigned int i = 0; i < m_numberOfJoints; ++i)
                m_maxDamping[i] = max_damping_bot.get(i+1).asDouble();
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
    if (m_pluginParameters.check("POSITION_CONTROL"))
    {
        if (setPIDsForGroup_POSITION(m_position_control_law, m_positionPIDs))
        {
        }
        else
        {
           yError() << "Error in one parameter of POSITION_CONTROL section";
           return false;
        }
    }
    else if (m_pluginParameters.check("GAZEBO_PIDS"))
    {
        yWarning ("'POSITION_CONTROL' group not found. Using DEPRECATED GAZEBO_PIDS section");
        setPIDsForGroup("GAZEBO_PIDS", m_positionPIDs, PIDFeedbackTermAllTerms);
        for (unsigned int i = 0; i < m_numberOfJoints; ++i) {m_position_control_law[i] = "joint_pid_gazebo_v1";}
    }
    else
    {
        yWarning() << "Unable to find a valid section containing position control gains, use default values";
        setPIDsForGroup("GAZEBO_PIDS", m_positionPIDs, PIDFeedbackTermAllTerms);
        for (unsigned int i = 0; i < m_numberOfJoints; ++i) {m_position_control_law[i] = "joint_pid_gazebo_v1";}
    }
    
    //VELOCITY PARAMETERS
    if (m_pluginParameters.check("VELOCITY_CONTROL"))
    {
        if (setPIDsForGroup_VELOCITY(m_velocity_control_law, m_velocityPIDs))
        {
        }
        else
        {
           yError() << "Error in one parameter of VELOCITY_CONTROL section";
           return false;
        }
    }
    else if (m_pluginParameters.check("GAZEBO_VELOCITY_PIDS"))
    { 
        yWarning ("'VELOCITY_CONTROL' group not found. Using DEPRECATED GAZEBO_VELOCITY_PIDS section");
        setPIDsForGroup("GAZEBO_VELOCITY_PIDS", m_velocityPIDs, PIDFeedbackTerm(PIDFeedbackTermProportionalTerm | PIDFeedbackTermIntegrativeTerm));
        for (unsigned int i = 0; i < m_numberOfJoints; ++i) {m_velocity_control_law[i] = "joint_pid_gazebo_v1";}
    }
    else
    {
        yWarning() << "Unable to find a valid section containing velocity control gains, use default values";
        setPIDsForGroup("GAZEBO_VELOCITY_PIDS", m_velocityPIDs, PIDFeedbackTerm(PIDFeedbackTermProportionalTerm | PIDFeedbackTermIntegrativeTerm));
        for (unsigned int i = 0; i < m_numberOfJoints; ++i) {m_velocity_control_law[i] = "joint_pid_gazebo_v1";}
    }

    //IMPEDANCE PARAMETERS
    if (m_pluginParameters.check("IMPEDANCE_CONTROL"))
    {
        if (setPIDsForGroup_IMPEDANCE(m_impedance_control_law, m_impedancePosPDs))
        {
        }
        else
        {
           yError() << "Error in one parameter of IMPEDANCE section";
           return false;
        }
    }
    else if (m_pluginParameters.check("GAZEBO_IMPEDANCE_POSITION_PIDS"))
    {
        yWarning ("'IMPEDANCE' group not found. Using DEPRECATED GAZEBO_PIDS section");
        setPIDsForGroup("GAZEBO_IMPEDANCE_POSITION_PIDS", m_impedancePosPDs, PIDFeedbackTerm(PIDFeedbackTermProportionalTerm | PIDFeedbackTermDerivativeTerm));
        for (unsigned int i = 0; i < m_numberOfJoints; ++i) {m_impedance_control_law[i] = "joint_pid_gazebo_v1";}
    }
    else
    {
        yWarning() << "Unable to find a valid section containing impedance control gains, use default values";
        setPIDsForGroup("GAZEBO_IMPEDANCE_POSITION_PIDS", m_impedancePosPDs, PIDFeedbackTerm(PIDFeedbackTermProportionalTerm | PIDFeedbackTermDerivativeTerm));
        for (unsigned int i = 0; i < m_numberOfJoints; ++i) {m_impedance_control_law[i] = "joint_pid_gazebo_v1";}
    }

    if (m_pluginParameters.check("SIMULATION"))
    {
        Bottle& simGroup = m_pluginParameters.findGroup("SIMULATION");
	Bottle xtmp;
        if (!validate(simGroup, xtmp, "kPWM", "kPWM parameter", m_numberOfJoints+1))  {yError() << "Missing kPWM parameter"; return false;} else {for (int j=0; j<m_numberOfJoints; j++) m_kPWM[j]=xtmp.get(j+1).asDouble();}
    }
    return true;
}

bool GazeboYarpControlBoardDriver::sendPositionsToGazebo(Vector &refs)
{
    for (unsigned int j=0; j<m_numberOfJoints; j++) {
        sendPositionToGazebo(j,refs[j]);
    }
    return true;
}

bool GazeboYarpControlBoardDriver::sendPositionToGazebo(int j,double ref)
{
    gazebo::msgs::JointCmd j_cmd;

    if(ref != m_oldReferencePositions[j])
    {
        // yDebug() << "Sending new command: new ref is " << ref << "; old ref was " << m_oldReferencePositions[j] ;
        prepareJointPositionMsg(j_cmd,j,ref);
        m_jointCommandPublisher->WaitForConnection();
        m_jointCommandPublisher->Publish(j_cmd);
        m_oldReferencePositions[j] = ref;
    }
    return true;
}

void GazeboYarpControlBoardDriver::prepareJointPositionMsg(gazebo::msgs::JointCmd& j_cmd, const int joint_index, const double ref)
{
    GazeboYarpControlBoardDriver::PID positionPID = m_positionPIDs[joint_index];

    j_cmd.set_name(m_jointPointers[joint_index]->GetScopedName());
    j_cmd.mutable_position()->set_target(convertUserToGazebo(joint_index, ref));
    j_cmd.mutable_position()->set_p_gain(positionPID.p);
    j_cmd.mutable_position()->set_i_gain(positionPID.i);
    j_cmd.mutable_position()->set_d_gain(positionPID.d);
    j_cmd.mutable_position()->set_i_max(positionPID.maxInt);
    j_cmd.mutable_position()->set_i_min(-positionPID.maxInt);
    j_cmd.mutable_position()->set_limit(positionPID.maxOut);
}

bool GazeboYarpControlBoardDriver::sendVelocitiesToGazebo(yarp::sig::Vector& refs) //NOT TESTED
{
    for (unsigned int j = 0; j < m_numberOfJoints; j++) {
        sendVelocityToGazebo(j,refs[j]);
    }
    return true;
}

bool GazeboYarpControlBoardDriver::sendVelocityToGazebo(int j,double ref)
{
    gazebo::msgs::JointCmd j_cmd;
    prepareJointVelocityMsg(j_cmd,j,ref);
    m_jointCommandPublisher->WaitForConnection();
    m_jointCommandPublisher->Publish(j_cmd);

    return true;
}

bool GazeboYarpControlBoardDriver::check_joint_within_limits_override_torque(int i, double& ref)
{
  double band =0;
  int   signKp = (m_positionPIDs[i].p>0?1:-1);
  int   signRef = (ref>0?1:-1);
  //if (m_controlMode[i] == VOCAB_CM_TORQUE || m_interactionMode[i] == VOCAB_IM_COMPLIANT)
  if (m_controlMode[i] != VOCAB_CM_IDLE && m_controlMode[i] != VOCAB_CM_FORCE_IDLE)
  {
      if  (m_positions[i] > m_jointPosLimits[i].max)
      {
          if (signKp*signRef >0 )
          {
              ref = ( (m_jointPosLimits[i].max-m_positions[i]) * (m_positionPIDs[i].p));
              if (ref > m_positionPIDs[i].maxOut) ref = m_positionPIDs[i].maxOut;
              else if (ref < -m_positionPIDs[i].maxOut) ref = -m_positionPIDs[i].maxOut;
              //_integral[i] = 0;
              #ifdef DEBUG_LIMITS
              yDebug() << "TTT TMAX" << m_positions[i] ">" <<  m_jointPosLimits[i].max;
              #endif
          }
          return false;
      }
      else if  (m_positions[i] < m_jointPosLimits[i].min)   
      {
          if (signKp*signRef <0 )
          {
              ref = ( (m_jointPosLimits[i].min-m_positions[i]) * (m_positionPIDs[i].p)); 
              if (ref > m_positionPIDs[i].maxOut) ref = m_positionPIDs[i].maxOut;
              else if (ref < -m_positionPIDs[i].maxOut) ref = -m_positionPIDs[i].maxOut;        
              //_integral[i] = 0;      
              #ifdef DEBUG_LIMITS
              yDebug() << "TTT TMIN" << m_positions[i] "<" <<  m_jointPosLimits[i].min;
              #endif
          }                               
          return false;
      }               
  }
  return true;
}
    
void GazeboYarpControlBoardDriver::prepareJointVelocityMsg(gazebo::msgs::JointCmd& j_cmd, const int j, const double ref) //NOT TESTED
{
    GazeboYarpControlBoardDriver::PID velocityPID = m_velocityPIDs[j];

    j_cmd.set_name(m_jointPointers[j]->GetScopedName());
    j_cmd.mutable_velocity()->set_p_gain(velocityPID.p);
    j_cmd.mutable_velocity()->set_i_gain(velocityPID.i);
    j_cmd.mutable_velocity()->set_d_gain(velocityPID.d);
    j_cmd.mutable_velocity()->set_i_max(velocityPID.maxInt);
    j_cmd.mutable_velocity()->set_i_min(-velocityPID.maxInt);
    j_cmd.mutable_velocity()->set_limit(velocityPID.maxOut);

    j_cmd.mutable_velocity()->set_target(convertUserToGazebo(j, ref));
}

bool GazeboYarpControlBoardDriver::sendTorquesToGazebo(yarp::sig::Vector& refs)
{
    for (unsigned int j = 0; j < m_numberOfJoints; j++) {
        sendTorqueToGazebo(j,refs[j]);
    }
    return true;
}

bool GazeboYarpControlBoardDriver::sendTorqueToGazebo(const int j, double ref) 
{
    if (check_joint_within_limits_override_torque(j,ref)==false)
    {
        //yDebug("TORQUE LIMITS %s j:%d ", deviceName.c_str(), j);      
    }
          
    gazebo::msgs::JointCmd j_cmd;
    prepareJointTorqueMsg(j_cmd,j,ref);
    m_jointCommandPublisher->WaitForConnection();
    m_jointCommandPublisher->Publish(j_cmd);
    return true;
}

void GazeboYarpControlBoardDriver::prepareJointTorqueMsg(gazebo::msgs::JointCmd& j_cmd, const int j, double ref)
{
    j_cmd.set_name(m_jointPointers[j]->GetScopedName());
//    j_cmd.mutable_position()->set_p_gain(0.0);
//    j_cmd.mutable_position()->set_i_gain(0.0);
//    j_cmd.mutable_position()->set_d_gain(0.0);
//    j_cmd.mutable_velocity()->set_p_gain(0.0);
//    j_cmd.mutable_velocity()->set_i_gain(0.0);
//    j_cmd.mutable_velocity()->set_d_gain(0.0);
    j_cmd.set_force(ref);
}

void GazeboYarpControlBoardDriver::sendImpPositionToGazebo ( const int j, double des )
{
    if(j >= 0 && j < m_numberOfJoints) {
        /*
         Here joint positions and speeds are in [deg] and [deg/sec].
         Therefore also stiffness and damping has to be [Nm/deg] and [Nm*sec/deg].
         */
        //yDebug()<<"m_velocities"<<j<<" : "<<m_velocities[j];
        double q = m_positions[j] - m_zeroPosition[j];
        double t_ref = -m_impedancePosPDs[j].p * (q - des) - m_impedancePosPDs[j].d * m_velocities[j] + m_torqueOffset[j];
        sendTorqueToGazebo(j, t_ref);
    }
}

void GazeboYarpControlBoardDriver::sendImpPositionsToGazebo ( Vector &dess )
{
    for(unsigned int i = 0; i < m_numberOfJoints; ++i)
        sendImpPositionToGazebo(i, dess[i]);
}


double GazeboYarpControlBoardDriver::convertGazeboToUser(int joint, gazebo::math::Angle value)
{
    double newValue = 0;
    switch(m_jointTypes[joint])
    {
        case JointType_Revolute:
        {
            newValue = value.Degree();
            break;
        }

        case JointType_Prismatic:
        {
            // For prismatic joints there is no getMeter() or something like that. The only way is to use .radiant() to get internal
            // value without changes
            newValue = value.Radian();
            break;
        }

        default:
        {
            yError() << "Cannot convert measure from Gazebo to User units, type of joint not supported for axes " <<
                                m_jointNames[joint] << " type is " << m_jointTypes[joint];
            break;
        }
    }
    return newValue;
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

        default:
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
    for(int i=0; i<m_numberOfJoints; i++)
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

        default:
        {
            yError() << "Cannot convert measure from User to Gazebo units, type of joint not supported";
            break;
        }
    }
    return newValue;
}

double * GazeboYarpControlBoardDriver::convertUserToGazebo(double *values)
{
    for(int i=0; i<m_numberOfJoints; i++)
        values[i] = convertGazeboToUser(i, values[i]);
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

        default:
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

        default:
        {
            yError() << "Cannot convert measure from Gazebo gains to User gain units, type of joint not supported for axes " <<
                                m_jointNames[joint] << " type is " << m_jointTypes[joint];
            break;
        }
    }
    return newValue;
}
