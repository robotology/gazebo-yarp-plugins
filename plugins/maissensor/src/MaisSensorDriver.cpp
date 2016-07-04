/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "MaisSensorDriver.h"
#include <GazeboYarpPlugins/common.h>

#include <cstdio>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/math/Angle.hh>

#include <yarp/os/LogStream.h>
#include <yarp/os/LockGuard.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;


GazeboYarpMaisSensorDriver::GazeboYarpMaisSensorDriver() : deviceName("") {}

GazeboYarpMaisSensorDriver::~GazeboYarpMaisSensorDriver() {}


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

bool GazeboYarpMaisSensorDriver::gazebo_init()
{
    //m_robot = gazebo_pointer_wrapper::getModel();
    // yDebug()<<"if this message is the last one you read, m_robot has not been set";
    //assert is a NOP in release mode. We should change the error handling either with an exception or something else
    assert(m_robot);
    if (!m_robot) return false;

    if (!setJointNames()) return false;      // this function also fills in the m_jointPointers vector

    m_channels_num = 16;
    m_numberOfJoints = m_jointNames.size();

    m_positions.resize(m_numberOfJoints);

    m_jointTypes.resize(m_numberOfJoints);

    // Initial zeroing of all vectors
    m_positions.zero();
    m_clock = 0;

    for (unsigned int j = 0; j < m_numberOfJoints; ++j)
    {
        m_jointTypes[j] = JointType_Unknown;
    }

    // This must be after zeroing of vectors
    if(!configureJointType() )
        return false;

    this->m_updateConnection =  gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboYarpMaisSensorDriver::onUpdate,  this, _1));

    m_gazeboNode = gazebo::transport::NodePtr(new gazebo::transport::Node);
    m_gazeboNode->Init(this->m_robot->GetWorld()->GetName());

    _T_controller = 1;

    return true;
}

bool GazeboYarpMaisSensorDriver::configureJointType()
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
                break;
            }

            case ( gazebo::physics::Entity::SLIDER_JOINT |  gazebo::physics::Entity::JOINT):
            {
                m_jointTypes[i] = JointType_Prismatic;
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

void GazeboYarpMaisSensorDriver::onUpdate(const gazebo::common::UpdateInfo& _info)
{
    LockGuard lock(m_mutex);
    m_clock++;

    // measurements acquisition
    for (unsigned int jnt_cnt = 0; jnt_cnt < m_jointPointers.size(); jnt_cnt++)
    {
        m_positions[jnt_cnt] = convertGazeboToUser(jnt_cnt, m_jointPointers[jnt_cnt]->GetAngle(0));
       // yDebug() << jnt_cnt << m_positions[jnt_cnt];
    }

    // Updating timestamp
    m_lastTimestamp.update(_info.simTime.Double());
}


bool GazeboYarpMaisSensorDriver::setJointNames()  //WORKS
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
    if (gazebo_models_joints.size() == 0)
    {
        yError() << "size of gazebo_models_joints is zero!";
        return false;
    }
    else
    {
        //yDebug() << " size of gazebo_models_joints: " <<gazebo_models_joints.size();
    }

    controlboard_joint_names.clear();
    for (unsigned int i = 0; i < m_jointNames.size(); i++) {
        bool joint_found = false;
        controlboard_joint_names.push_back(joint_names_bottle.get(i+1).asString().c_str());

        for (unsigned int gazebo_joint = 0; gazebo_joint < gazebo_models_joints.size() && !joint_found; gazebo_joint++)
        {
            std::string gazebo_joint_name = gazebo_models_joints[gazebo_joint]->GetName();
            
            //char buff[1000];
            //sprintf(buff, "full:'%s' sub:'%s'", gazebo_joint_name.c_str(),controlboard_joint_names[i].c_str());
            //yDebug() << "***" << buff;
            
            if (GazeboYarpPlugins::hasEnding(gazebo_joint_name,controlboard_joint_names[i]))
            {
                joint_found = true;
                m_jointNames[i] = gazebo_joint_name;
                m_jointPointers[i] = this->m_robot->GetJoint(gazebo_joint_name);
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


double GazeboYarpMaisSensorDriver::convertGazeboToUser(int joint, gazebo::math::Angle value)
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

double GazeboYarpMaisSensorDriver::convertGazeboToUser(int joint, double value)
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

double * GazeboYarpMaisSensorDriver::convertGazeboToUser(double *values)
{
    for(int i=0; i<m_numberOfJoints; i++)
        values[i] = convertGazeboToUser(i, values[i]);
    return values;
}

double GazeboYarpMaisSensorDriver::convertUserToGazebo(int joint, double value)
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

double * GazeboYarpMaisSensorDriver::convertUserToGazebo(double *values)
{
    for(int i=0; i<m_numberOfJoints; i++)
        values[i] = convertGazeboToUser(i, values[i]);
    return values;
}


int GazeboYarpMaisSensorDriver::read(yarp::sig::Vector &out)
{
    LockGuard lock(m_mutex);
    out = m_positions;
    return yarp::dev::IAnalogSensor::AS_OK;
}

int GazeboYarpMaisSensorDriver::getState(int ch)
{
    LockGuard lock(m_mutex);
    return yarp::dev::IAnalogSensor::AS_OK;
}

int GazeboYarpMaisSensorDriver::getChannels()
{
    LockGuard lock(m_mutex);
    return m_channels_num;
}

int GazeboYarpMaisSensorDriver::calibrateSensor()
{
    LockGuard lock(m_mutex);
    // not implemented
    return 0; 
}

int GazeboYarpMaisSensorDriver::calibrateSensor(const yarp::sig::Vector& value)
{
    LockGuard lock(m_mutex);
    // not implemented
    return 0;
}

int GazeboYarpMaisSensorDriver::calibrateChannel(int ch)
{
    LockGuard lock(m_mutex);
    // not implemented
    return 0;
}

int GazeboYarpMaisSensorDriver::calibrateChannel(int ch, double value)
{
    LockGuard lock(m_mutex);
    // not implemented
    return 0;
}
