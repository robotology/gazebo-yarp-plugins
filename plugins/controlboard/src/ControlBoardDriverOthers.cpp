/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "ControlBoardDriver.h"
#include <gazebo/physics/physics.hh>

using namespace yarp::dev;

bool GazeboYarpControlBoardDriver::getAxisName(int axis, yarp::os::ConstString& name)
{
    if (axis < 0 || axis >= (int)m_numberOfJoints) return false;
    name =  yarp::os::ConstString(controlboard_joint_names.at(axis));
    return true;
}

bool GazeboYarpControlBoardDriver::getJointType(int axis, yarp::dev::JointTypeEnum& type)
{
    if (axis < 0 || axis >= (int)m_numberOfJoints) return false;
    if (this->m_jointTypes[axis] == JointType_Revolute    ) {
        type = yarp::dev::VOCAB_JOINTTYPE_REVOLUTE;
    } else if (this->m_jointTypes[axis] == JointType_Prismatic ) {
        type = yarp::dev::VOCAB_JOINTTYPE_PRISMATIC;
    } else {
      yarp::dev::VOCAB_JOINTTYPE_UNKNOWN;
    }
    
    yarp::os::ConstString(controlboard_joint_names.at(axis));
    return true;
}

// IControlLimits
bool GazeboYarpControlBoardDriver::getLimits(int axis, double *min, double *max) //WORKS
{
    if (axis < 0 || axis >= (int)m_numberOfJoints) return false;
    if (!min || !max) return false;
    *min = m_jointPosLimits[axis].min;
    *max = m_jointPosLimits[axis].max;
    return true;
}

bool GazeboYarpControlBoardDriver::setLimits(int axis, double min, double max) //WORKS
{
    if (axis < 0 || axis >= (int)m_numberOfJoints) return false;
    m_jointPosLimits[axis].max = max;
    m_jointPosLimits[axis].min = min;
    return true;
}

// IControlLimits2
bool GazeboYarpControlBoardDriver::getVelLimits(int axis, double* min, double* max) //WORKS
{
    if (axis < 0 || axis >= (int)m_numberOfJoints) return false;
    if (!min || !max) return false;
    *min = m_jointVelLimits[axis].min;
    *max = m_jointVelLimits[axis].max;
    return true;
}

bool GazeboYarpControlBoardDriver::setVelLimits(int axis, double min, double max) //WORKS
{
    if (axis < 0 || axis >= (int)m_numberOfJoints) return false;
    m_jointVelLimits[axis].max = max;
    m_jointVelLimits[axis].min = min;
    return true;
}

//Amplifiers
bool GazeboYarpControlBoardDriver::enableAmp(int j) //NOT IMPLEMENTED
{
    if (j >= 0 && j < (int)m_numberOfJoints) {
        amp[j] = 1;
        m_controlMode[j] = VOCAB_CM_POSITION;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::disableAmp(int j) //NOT IMPLEMENTED
{
    if (j >= 0 && j < (int)m_numberOfJoints) {
        amp[j] = 0;
        m_controlMode[j] = VOCAB_CM_IDLE;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getCurrent(int j, double* val) //NOT IMPLEMENTED
{
    if (val && j >= 0 && j < (int)m_numberOfJoints) {
        *val = amp[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getCurrents(double *vals) //NOT IMPLEMENTED
{
    if (!vals) return false;
    for (unsigned int i=0; i<m_numberOfJoints; i++) {
        vals[i] = amp[i];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::setMaxCurrent(int, double) //NOT IMPLEMENTED
{
    return true;
}

bool GazeboYarpControlBoardDriver::getMaxCurrent(int j, double *v) //NOT IMPLEMENTED
{
    if (!v) return false;
    *v = 0;
    return true;
}

bool GazeboYarpControlBoardDriver::getAmpStatus(int *st) //NOT IMPLEMENTED
{
    if (!st) return false;
    *st = 0;
    return true;
}

bool GazeboYarpControlBoardDriver::getAmpStatus(int, int *v) //NOT IMPLEMENTED
{
    if (!v) return false;
    *v = 0;
    return true;
}

bool GazeboYarpControlBoardDriver::calibrate2(int j, unsigned int iv, double v1, double v2, double v3) //NOT IMPLEMENTED
{
    yDebug("fakebot: calibrating joint %d with parameters %u %f %f %f\n", j, iv, v1, v2, v3);
    return true;
}

bool GazeboYarpControlBoardDriver::done(int j) // NOT IMPLEMENTED
{
    yDebug("fakebot: calibration done on joint %d.\n", j);
    return true;
}

bool GazeboYarpControlBoardDriver::getRemoteVariablesList(yarp::os::Bottle* listOfKeys)
{
    listOfKeys->clear();
    listOfKeys->addString("hardwareDamping");
    listOfKeys->addString("hardwareEffortLimit");
    listOfKeys->addString("hardwareVelocityLimit");
    listOfKeys->addString("yarp_jntMaxVel");
    listOfKeys->addString("SHORTCUT_all_pos_kp");
    listOfKeys->addString("SHORTCUT_all_pos_kd");
    listOfKeys->addString("SHORTCUT_all_pos_ki");
        
    return true;
}

bool GazeboYarpControlBoardDriver::getRemoteVariable(yarp::os::ConstString key, yarp::os::Bottle& val)
{
    val.clear();
    if (key == "hardwareDamping")
    {
        yarp::os::Bottle& r = val.addList(); for (int i = 0; i< m_numberOfJoints; i++) { double tmp = m_jointPointers[i]->GetDamping(0);  r.addDouble(tmp); }
        return true;
    }
    if (key == "hardwareEffortLimit")
    {
        yarp::os::Bottle& r = val.addList(); for (int i = 0; i< m_numberOfJoints; i++) { double tmp = m_jointPointers[i]->GetEffortLimit(0);  r.addDouble(tmp); }
        return true;
    }
    if (key == "hardwareVelocityLimit")
    {
        yarp::os::Bottle& r = val.addList(); for (int i = 0; i< m_numberOfJoints; i++) { double tmp = m_jointPointers[i]->GetVelocityLimit(0);  r.addDouble(tmp); }
        return true;
    }
    if (key == "yarp_jntMaxVel")
    {
        yarp::os::Bottle& r = val.addList(); for (int i = 0; i< m_numberOfJoints; i++) { double tmp_min,tmp_max; getVelLimits(i,&tmp_min,&tmp_max);  r.addDouble(tmp_max); }
        return true;
    }
    if (key == "SHORTCUT_all_pos_kp")
    {   
        yarp::os::Bottle& r = val.addList(); for (int i = 0; i< m_numberOfJoints; i++) { yarp::dev::Pid tmp_pid; getPid(i,&tmp_pid);  r.addDouble(tmp_pid.kp); }
        return true;
    }
    if (key == "SHORTCUT_all_pos_kd")
    {   
        yarp::os::Bottle& r = val.addList(); for (int i = 0; i< m_numberOfJoints; i++) { yarp::dev::Pid tmp_pid; getPid(i,&tmp_pid);  r.addDouble(tmp_pid.kd); }
        return true;
    }
    if (key == "SHORTCUT_all_pos_ki")
    {   
        yarp::os::Bottle& r = val.addList(); for (int i = 0; i< m_numberOfJoints; i++) { yarp::dev::Pid tmp_pid; getPid(i,&tmp_pid);  r.addDouble(tmp_pid.ki); }
        return true;
    }
    yWarning("getRemoteVariable(): Unknown variable %s", key.c_str());
    return false;
}

bool GazeboYarpControlBoardDriver::setRemoteVariable(yarp::os::ConstString key, const yarp::os::Bottle& val)
{
    std::string s1 = val.toString();
    yarp::os::Bottle* bval = val.get(0).asList();
    if (bval == 0)
    {
        yWarning("setRemoteVariable(): Protocol error %s", s1.c_str());
        return false;
    }

    std::string s2 = bval->toString();

    if (key == "hardwareDamping")
    {
        for (int i = 0; i < m_numberOfJoints; i++)
        {
            double value = bval->get(i).asInt();
            m_jointPointers[i]->SetDamping(0,value);
        }
        return true;
    }
    if (key == "hardwareEffortLimit")
    {
        for (int i = 0; i < m_numberOfJoints; i++)
        {
            double value = bval->get(i).asInt();
            m_jointPointers[i]->SetEffortLimit(0,value);
        }
        return true;
    }
    if (key == "hardwareVelocityLimit")
    {
        for (int i = 0; i < m_numberOfJoints; i++)
        {
            double value = bval->get(i).asInt();
            m_jointPointers[i]->SetVelocityLimit(0,value);
        }
        return true;
    }
    if (key == "yarp_jntMaxVel")
    {
        for (int i = 0; i < m_numberOfJoints; i++)
        {
            double value = bval->get(i).asInt();
            setVelLimits(i,0,value);
        }
        return true;
    }
    if (key == "SHORTCUT_all_pos_kp")
    {
        for (int i = 0; i < m_numberOfJoints; i++)
        {
            yarp::dev::Pid tmp_pid;
            getPid(i,&tmp_pid);
            tmp_pid.kp = bval->get(i).asInt();
            setPid(i,tmp_pid);
        }
        return true;
    }
    if (key == "SHORTCUT_all_pos_kd")
    {
        for (int i = 0; i < m_numberOfJoints; i++)
        {
            yarp::dev::Pid tmp_pid;
            getPid(i,&tmp_pid);
            tmp_pid.kd = bval->get(i).asInt();
            setPid(i,tmp_pid);
        }
        return true;
    }
    if (key == "SHORTCUT_all_pos_ki")
    {
        for (int i = 0; i < m_numberOfJoints; i++)
        {
            yarp::dev::Pid tmp_pid;
            getPid(i,&tmp_pid);
            tmp_pid.ki = bval->get(i).asInt();
            setPid(i,tmp_pid);
        }
        return true;
    }
    yWarning("setRemoteVariable(): Unknown variable %s", key.c_str());
    return false;
}

