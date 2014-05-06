/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include "gazebo_yarp_plugins/ControlBoardDriver.h"
#include "gazebo_yarp_plugins/common.h"
#include <stdio.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/math/Angle.hh>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

const double RobotPositionTolerance = 0.9;

GazeboYarpControlBoardDriver::GazeboYarpControlBoardDriver() {}
GazeboYarpControlBoardDriver::~GazeboYarpControlBoardDriver() {}

bool GazeboYarpControlBoardDriver::gazebo_init()
{
    //m_robot = gazebo_pointer_wrapper::getModel();
    // std::cout<<"if this message is the last one you read, m_robot has not been set"<<std::endl;
    //assert is a NOP in release mode. We should change the error handling either with an exception or something else
    assert(m_robot);
    if (!m_robot) return false;

    std::cout<<"Robot Name: "<<m_robot->GetName() <<std::endl;
    std::cout<<"# Joints: "<<m_robot->GetJoints().size() <<std::endl;
    std::cout<<"# Links: "<<m_robot->GetLinks().size() <<std::endl;

    this->m_robotRefreshPeriod = this->m_robot->GetWorld()->GetPhysicsEngine()->GetUpdatePeriod() * 1000.0;
    if (!setJointNames()) return false;

    m_numberOfJoints = m_jointNames.size();
    m_positions.resize(m_numberOfJoints);
    m_zeroPosition.resize(m_numberOfJoints);
    m_referenceVelocities.resize(m_numberOfJoints);
    m_velocities.resize(m_numberOfJoints);
    amp.resize(m_numberOfJoints);
    m_torques.resize(m_numberOfJoints); m_torques.zero();
    m_trajectoryGenerationReferenceSpeed.resize(m_numberOfJoints);
    m_referencePositions.resize(m_numberOfJoints);
    m_trajectoryGenerationReferencePosition.resize(m_numberOfJoints);
    m_trajectoryGenerationReferenceAcceleraton.resize(m_numberOfJoints);
    m_referenceTorques.resize(m_numberOfJoints);
    m_jointLimits.resize(m_numberOfJoints);
    m_positionPIDs.reserve(m_numberOfJoints);
    m_velocityPIDs.reserve(m_numberOfJoints);
    m_impedancePosPDs.reserve(m_numberOfJoints);
    m_torqueOffsett.resize(m_numberOfJoints);
    m_minStiffness.resize(m_numberOfJoints, 0.0);
    m_maxStiffness.resize(m_numberOfJoints, 1000.0);
    m_minDamping.resize(m_numberOfJoints, 0.0);
    m_maxDamping.resize(m_numberOfJoints, 100.0);

    setMinMaxPos();
    setMinMaxImpedance();
    setPIDs();
    m_positions.zero();
    m_zeroPosition.zero();
    m_referenceVelocities.zero();
    m_velocities.zero();
    m_trajectoryGenerationReferenceSpeed.zero();
    m_referencePositions.zero();
    m_trajectoryGenerationReferencePosition.zero();
    m_trajectoryGenerationReferenceAcceleraton.zero();
    m_referenceTorques.zero();
    amp = 1; // initially on - ok for simulator
    started = false;
    m_controlMode = new int[m_numberOfJoints];
    m_isMotionDone = new bool[m_numberOfJoints];
    m_clock = 0;
    m_torqueOffsett = 0;
    for (unsigned int j = 0; j < m_numberOfJoints; ++j)
        m_controlMode[j] = VOCAB_CM_POSITION;

    std::cout << "gazebo_init set pid done!" << std::endl;

    this->m_updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(                                                                           boost::bind(&GazeboYarpControlBoardDriver::onUpdate, this, _1));

    m_gazeboNode = gazebo::transport::NodePtr(new gazebo::transport::Node);
    m_gazeboNode->Init(this->m_robot->GetWorld()->GetName());
    m_jointCommandPublisher = m_gazeboNode->Advertise<gazebo::msgs::JointCmd>(std::string("~/") + this->m_robot->GetName() + "/joint_cmd");

    _T_controller = 1;

    std::stringstream ss(m_pluginParameters.find("initialConfiguration").toString());
    if (!(m_pluginParameters.find("initialConfiguration") == "")) {
        double tmp = 0.0;
        yarp::sig::Vector initial_config(m_numberOfJoints);
        unsigned int counter = 1;
        while (ss >> tmp) {
            if(counter > m_numberOfJoints) {
                std::cout<<"To many element in initial configuration, stopping at element "<<counter<<std::endl;
                break;
            }
            initial_config[counter-1] = tmp;
            m_trajectoryGenerationReferencePosition[counter - 1] = GazeboYarpPlugins::convertRadiansToDegrees(tmp);
            m_referencePositions[counter - 1] = GazeboYarpPlugins::convertRadiansToDegrees(tmp);
            m_positions[counter - 1] = GazeboYarpPlugins::convertRadiansToDegrees(tmp);
            counter++;
        }
        std::cout<<"INITIAL CONFIGURATION IS: "<<initial_config.toString()<<std::endl;

        for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
            gazebo::math::Angle a;
            a.SetFromRadian(initial_config[i]);
            std::string joint_name = m_jointNames[i];
            m_robot->GetJoint(joint_name)->SetAngle(0,a);
        }
    }
    return true;
}

void GazeboYarpControlBoardDriver::computeTrajectory(const int j)
{
    if ((m_referencePositions[j] - m_trajectoryGenerationReferencePosition[j]) < -RobotPositionTolerance) {
        if (m_trajectoryGenerationReferenceSpeed[j] !=0)
            m_referencePositions[j] += (m_trajectoryGenerationReferenceSpeed[j] / 1000.0) * m_robotRefreshPeriod * _T_controller;
        m_isMotionDone[j] = false;
    } else if ((m_referencePositions[j] - m_trajectoryGenerationReferencePosition[j]) > RobotPositionTolerance) {
        if (m_trajectoryGenerationReferenceSpeed[j] != 0)
            m_referencePositions[j]-= (m_trajectoryGenerationReferenceSpeed[j] / 1000.0) * m_robotRefreshPeriod * _T_controller;
        m_isMotionDone[j] = false;
    } else {
        m_referencePositions[j] = m_trajectoryGenerationReferencePosition[j];
        m_isMotionDone[j] = true;
    }
}

void GazeboYarpControlBoardDriver::onUpdate(const gazebo::common::UpdateInfo& /*_info*/)
{
    m_clock++;

    if (!started) {//This is a simple way to start with the robot in standing position
        started = true;
        for (unsigned int j = 0; j < m_numberOfJoints; ++j)
            sendPositionToGazebo (j, m_positions[j]);
    }

    // Sensing position & torque
    for (unsigned int jnt_cnt = 0; jnt_cnt < m_jointNames.size(); jnt_cnt++) {
//TODO: consider multi-dof joint ?
        m_positions[jnt_cnt] = this->m_robot->GetJoint(m_jointNames[jnt_cnt])->GetAngle (0).Degree();
        m_velocities[jnt_cnt] = GazeboYarpPlugins::convertRadiansToDegrees(this->m_robot->GetJoint(m_jointNames[jnt_cnt])->GetVelocity(0));
        m_torques[jnt_cnt] = this->m_robot->GetJoint(m_jointNames[jnt_cnt])->GetForce(0);
    }

    //logger.log(m_velocities[2]);

    for (unsigned int j = 0; j < m_numberOfJoints; ++j) {
        if (m_controlMode[j] == VOCAB_CM_POSITION) {//set pos joint value, set m_referenceVelocities joint value
            if (m_clock % _T_controller == 0) {
                computeTrajectory(j);
                sendPositionToGazebo(j, m_referencePositions[j]);
            }
        } else if (m_controlMode[j] == VOCAB_CM_VELOCITY) {//set vmo joint value
            if (m_clock % _T_controller == 0) {
                sendVelocityToGazebo(j, m_referenceVelocities[j]);
            }
        } else if (m_controlMode[j] == VOCAB_CM_TORQUE) {
            if (m_clock % _T_controller == 0) {
                sendTorqueToGazebo(j, m_referenceTorques[j]);
            }
        } else if (m_controlMode[j] == VOCAB_CM_OPENLOOP) {
            //OpenLoop control sends torques to gazebo at this moment.
            //Check if gazebo implements a "motor" entity and change the code accordingly.
            if (m_clock % _T_controller == 0) {
                sendTorqueToGazebo(j, m_referenceTorques[j]);
            }
        } else if ( m_controlMode[j] == VOCAB_CM_IMPEDANCE_POS) {
            if (m_clock % _T_controller == 0) {
                computeTrajectory(j);
                sendImpPositionToGazebo(j, m_referencePositions[j]);
            }
        }
    }
}

void GazeboYarpControlBoardDriver::setMinMaxPos()  //NOT TESTED
{
    std::cout<<"Joint Limits"<<std::endl;
    for(unsigned int i = 0; i < m_numberOfJoints; ++i) {
        m_jointLimits[i].max = this->m_robot->GetJoint(m_jointNames[i])->GetUpperLimit(0).Degree();
        m_jointLimits[i].min = this->m_robot->GetJoint(m_jointNames[i])->GetLowerLimit(0).Degree();
        std::cout<<m_jointNames[i]<<" max_pos: "<< m_jointLimits[i].max<<" min_pos: "<< m_jointLimits[i].min <<std::endl;
    }
}

bool GazeboYarpControlBoardDriver::setJointNames()  //WORKS
{
    std::cout << ".ini file found, using joint names in ini file" << std::endl;
    yarp::os::Bottle joint_names_bottle = m_pluginParameters.findGroup("jointNames");

    if (joint_names_bottle.isNull()) {
        std::cout << "GazeboYarpControlBoardDriver::setJointNames(): Error cannot find jointNames." << std::endl;
        return false;
    }

    int nr_of_joints = joint_names_bottle.size()-1;

    m_jointNames.resize(nr_of_joints);

    const gazebo::physics::Joint_V & gazebo_models_joints = m_robot->GetJoints();

    for (unsigned int i = 0; i < m_jointNames.size(); i++) {
        bool joint_found = false;
        std::string controlboard_joint_name(joint_names_bottle.get(i+1).asString().c_str());

        for (unsigned int gazebo_joint = 0; gazebo_joint < gazebo_models_joints.size() && !joint_found; gazebo_joint++) {
            std::string gazebo_joint_name = gazebo_models_joints[gazebo_joint]->GetName();
            if (GazeboYarpPlugins::hasEnding(gazebo_joint_name,controlboard_joint_name)) {
                joint_found = true;
                m_jointNames[i] = gazebo_joint_name;
            }
        }

        if (!joint_found) {
            std::cout << "GazeboYarpControlBoardDriver::setJointNames(): Error, cannot find joint " << m_jointNames[i] << std::endl;
            m_jointNames.resize(0);
            return false;
        }
    }
    return true;
}

void GazeboYarpControlBoardDriver::setPIDsForGroup(std::string pidGroupName,
                                                   std::vector<GazeboYarpControlBoardDriver::PID>& pids,
                                                   enum PIDFeedbackTerm pidTerms)
{
    yarp::os::Property prop;
    if (m_pluginParameters.check(pidGroupName.c_str())) {
        std::cout<<"Found PID information in plugin parameters group " << pidGroupName << std::endl;

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
            std::cout<<"  P: "<<pidValue.p<<" I: "<<pidValue.i<<" D: "<<pidValue.d<<" maxInt: "<<pidValue.maxInt<<" maxOut: "<<pidValue.maxOut<<std::endl;
        }
        std::cout<<"OK!"<<std::endl;
    } else {
        double default_p = pidTerms & PIDFeedbackTermProportionalTerm ? 500.0 : 0;
        double default_i = pidTerms & PIDFeedbackTermIntegrativeTerm ? 0.1 : 0;
        double default_d = pidTerms & PIDFeedbackTermDerivativeTerm ? 1.0 : 0;
        std::cout<<"PID gain information not found in plugin parameters, using default gains ( "
        <<"P " << default_p << " I " << default_i << " D " << default_d << " )" <<std::endl;
        for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
            GazeboYarpControlBoardDriver::PID pid = {500, 0.1, 1.0, -1, -1};
            pids.push_back(pid);
        }
    }
}

void GazeboYarpControlBoardDriver::setMinMaxImpedance()
{

    yarp::os::Bottle& name_bot = m_pluginParameters.findGroup("WRAPPER").findGroup("networks");
    std::string name = name_bot.get(1).toString();

    yarp::os::Bottle& kin_chain_bot = m_pluginParameters.findGroup(name);
    if (kin_chain_bot.check("min_stiffness")) {
        std::cout<<"min_stiffness param found!"<<std::endl;
        yarp::os::Bottle& min_stiff_bot = kin_chain_bot.findGroup("min_stiffness");
        if(min_stiff_bot.size()-1 == m_numberOfJoints) {
            for(unsigned int i = 0; i < m_numberOfJoints; ++i)
                m_minStiffness[i] = min_stiff_bot.get(i+1).asDouble();
        } else
            std::cout<<"Invalid number of params"<<std::endl;
    } else
        std::cout<<"No minimum stiffness value found in ini file, default one will be used!"<<std::endl;

    if (kin_chain_bot.check("max_stiffness")) {
        std::cout<<"max_stiffness param found!"<<std::endl;
        yarp::os::Bottle& max_stiff_bot = kin_chain_bot.findGroup("max_stiffness");
        if (max_stiff_bot.size()-1 == m_numberOfJoints) {
            for (unsigned int i = 0; i < m_numberOfJoints; ++i)
                m_maxStiffness[i] = max_stiff_bot.get(i+1).asDouble();
        } else
            std::cout<<"Invalid number of params"<<std::endl;
    }
    else
        std::cout<<"No maximum stiffness value found in ini file, default one will be used!"<<std::endl;

    if (kin_chain_bot.check("min_damping")) {
        std::cout<<"min_damping param found!"<<std::endl;
        yarp::os::Bottle& min_damping_bot = kin_chain_bot.findGroup("min_damping");
        if(min_damping_bot.size()-1 == m_numberOfJoints) {
            for(unsigned int i = 0; i < m_numberOfJoints; ++i)
                m_minDamping[i] = min_damping_bot.get(i+1).asDouble();
        } else
            std::cout<<"Invalid number of params"<<std::endl;
    } else
        std::cout<<"No minimum dampings value found in ini file, default one will be used!"<<std::endl;

    if(kin_chain_bot.check("max_damping")) {
        std::cout<<"max_damping param found!"<<std::endl;
        yarp::os::Bottle& max_damping_bot = kin_chain_bot.findGroup("max_damping");
        if (max_damping_bot.size() - 1 == m_numberOfJoints) {
            for(unsigned int i = 0; i < m_numberOfJoints; ++i)
                m_maxDamping[i] = max_damping_bot.get(i+1).asDouble();
        } else
            std::cout<<"Invalid number of params"<<std::endl;
    } else
        std::cout<<"No maximum damping value found in ini file, default one will be used!"<<std::endl;

    std::cout<<"min_stiffness: [ "<<m_minStiffness.toString()<<" ]"<<std::endl;
    std::cout<<"max_stiffness: [ "<<m_maxStiffness.toString()<<" ]"<<std::endl;
    std::cout<<"min_damping: [ "<<m_minDamping.toString()<<" ]"<<std::endl;
    std::cout<<"max_damping: [ "<<m_maxDamping.toString()<<" ]"<<std::endl;
}

void GazeboYarpControlBoardDriver::setPIDs()
{
    setPIDsForGroup("GAZEBO_PIDS", m_positionPIDs, PIDFeedbackTermAllTerms);
    setPIDsForGroup("GAZEBO_VELOCITY_PIDS", m_velocityPIDs, PIDFeedbackTerm(PIDFeedbackTermProportionalTerm | PIDFeedbackTermIntegrativeTerm));
    setPIDsForGroup("GAZEBO_IMPEDANCE_POSITION_PIDS", m_impedancePosPDs, PIDFeedbackTerm(PIDFeedbackTermProportionalTerm | PIDFeedbackTermDerivativeTerm));
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
    prepareJointMsg(j_cmd,j,ref);
    m_jointCommandPublisher->WaitForConnection();
    m_jointCommandPublisher->Publish(j_cmd);
    return true;
}

void GazeboYarpControlBoardDriver::prepareJointMsg(gazebo::msgs::JointCmd& j_cmd, const int joint_index, const double ref)  //WORKS
{
    GazeboYarpControlBoardDriver::PID positionPID = m_positionPIDs[joint_index];

    j_cmd.set_name(this->m_robot->GetJoint(m_jointNames[joint_index])->GetScopedName());
    j_cmd.mutable_position()->set_target(GazeboYarpPlugins::convertDegreesToRadians(ref));
    j_cmd.mutable_position()->set_p_gain(positionPID.p);
    j_cmd.mutable_position()->set_i_gain(positionPID.i);
    j_cmd.mutable_position()->set_d_gain(positionPID.d);
}

bool GazeboYarpControlBoardDriver::sendVelocitiesToGazebo(yarp::sig::Vector& refs) //NOT TESTED
{
    for (unsigned int j = 0; j < m_numberOfJoints; j++) {
        sendVelocityToGazebo(j,refs[j]);
    }
    return true;
}

bool GazeboYarpControlBoardDriver::sendVelocityToGazebo(int j,double ref) //NOT TESTED
{
    /* SetVelocity method */
    /*gazebo::physics::JointPtr joint =  this->m_robot->GetJoint(m_jointNames[j]);
     joint->SetMaxForce(0, joint->GetEffortLimit(0)*1.1); //<-- MAGIC NUMBER!!!!
     //      std::cout<<"MaxForce:" <<joint->GetMaxForce(0)<<std::endl;
     joint->SetVelocity(0,toRad(ref));
     */
    /* JointController method. If you pick this control method for control
     *      of joint velocities, you should also take care of the switching logic
     *      in setVelocityMode, setTorqueMode and setPositionMode:
     *      that is, the SetMarxForce(0,0) and SetVelocity(0,0) are no longer
     *      needed, but the JointController::AddJoint() method needs to be called
     *      when you switch to velocity mode, to make sure the PIDs get reset */
    gazebo::msgs::JointCmd j_cmd;
    prepareJointVelocityMsg(j_cmd,j,ref);
    m_jointCommandPublisher->WaitForConnection();
    m_jointCommandPublisher->Publish(j_cmd);
    /**/
    return true;
}

void GazeboYarpControlBoardDriver::prepareJointVelocityMsg(gazebo::msgs::JointCmd& j_cmd, const int j, const double ref) //NOT TESTED
{
    GazeboYarpControlBoardDriver::PID velocityPID = m_velocityPIDs[j];

    j_cmd.set_name(this->m_robot->GetJoint(m_jointNames[j])->GetScopedName());
//    j_cmd.mutable_position()->set_p_gain(0.0);
//    j_cmd.mutable_position()->set_i_gain(0.0);
//    j_cmd.mutable_position()->set_d_gain(0.0);
    j_cmd.mutable_velocity()->set_p_gain(velocityPID.p);
    j_cmd.mutable_velocity()->set_i_gain(velocityPID.i);
    j_cmd.mutable_velocity()->set_d_gain(velocityPID.d);
    //     if (velocityPID.maxInt > 0) {
    //         j_cmd.mutable_velocity()->set_i_max(velocityPID.maxInt);
    //         j_cmd.mutable_velocity()->set_i_min(-velocityPID.maxInt);
    //     }
    //     if (velocityPID.maxOut > 0) {
    //         j_cmd.mutable_velocity()->set_limit(velocityPID.maxOut);
    //     }

    j_cmd.mutable_velocity()->set_target(GazeboYarpPlugins::convertDegreesToRadians(ref));
}

bool GazeboYarpControlBoardDriver::sendTorquesToGazebo(yarp::sig::Vector& refs) //NOT TESTED
{
    for (unsigned int j = 0; j < m_numberOfJoints; j++) {
        sendTorqueToGazebo(j,refs[j]);
    }
    return true;
}

bool GazeboYarpControlBoardDriver::sendTorqueToGazebo(const int j,const double ref) //NOT TESTED
{
    gazebo::msgs::JointCmd j_cmd;
    prepareJointTorqueMsg(j_cmd,j,ref);
    m_jointCommandPublisher->WaitForConnection();
    m_jointCommandPublisher->Publish(j_cmd);
    return true;
}

void GazeboYarpControlBoardDriver::prepareJointTorqueMsg(gazebo::msgs::JointCmd& j_cmd, const int j, const double ref) //NOT TESTED
{
    j_cmd.set_name(this->m_robot->GetJoint(m_jointNames[j])->GetScopedName());
//    j_cmd.mutable_position()->set_p_gain(0.0);
//    j_cmd.mutable_position()->set_i_gain(0.0);
//    j_cmd.mutable_position()->set_d_gain(0.0);
//    j_cmd.mutable_velocity()->set_p_gain(0.0);
//    j_cmd.mutable_velocity()->set_i_gain(0.0);
//    j_cmd.mutable_velocity()->set_d_gain(0.0);
    j_cmd.set_force(ref);
}

void GazeboYarpControlBoardDriver::sendImpPositionToGazebo ( const int j, const double des )
{
    if(j >= 0 && j < m_numberOfJoints) {
        /*
         Here joint positions and speeds are in [deg] and [deg/sec].
         Therefore also stiffness and damping has to be [Nm/deg] and [Nm*sec/deg].
         */
        //std::cout<<"m_velocities"<<j<<" : "<<m_velocities[j]<<std::endl;
        double q = m_positions[j] - m_zeroPosition[j];
        double t_ref = -m_impedancePosPDs[j].p * (q - des) - m_impedancePosPDs[j].d * m_velocities[j] + m_torqueOffsett[j];
        sendTorqueToGazebo(j, t_ref);
    }
}

void GazeboYarpControlBoardDriver::sendImpPositionsToGazebo ( Vector &dess )
{
    for(unsigned int i = 0; i < m_numberOfJoints; ++i)
        sendImpPositionToGazebo(i, dess[i]);
}
