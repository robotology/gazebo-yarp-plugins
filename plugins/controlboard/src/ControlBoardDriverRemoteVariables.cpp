/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "ControlBoardDriver.h"
#include <gazebo/physics/Joint.hh>

using namespace yarp::dev;

bool GazeboYarpControlBoardDriver::getRemoteVariablesList(yarp::os::Bottle* listOfKeys)
{
    listOfKeys->clear();
    listOfKeys->addString("hardwareDamping");
    listOfKeys->addString("hardwareFriction");
    listOfKeys->addString("hardwareEffortLimit");
    
    listOfKeys->addString("hardwareVelocityLimit");
    listOfKeys->addString("yarp_jntMaxVel");
    
    listOfKeys->addString("yarp_jntMaxPos");
    listOfKeys->addString("yarp_jntMinPos");
    listOfKeys->addString("yarp_kPWM");
	
    listOfKeys->addString("hardwareHiStop");
    listOfKeys->addString("hardwareLowStop");
    
    listOfKeys->addString("SHORTCUT_all_pos_kp");
    listOfKeys->addString("SHORTCUT_all_pos_kd");
    listOfKeys->addString("SHORTCUT_all_pos_ki");
        
    listOfKeys->addString("VelocityTimeout");
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
    if (key == "hardwareFriction")
    {
        yarp::os::Bottle& r = val.addList(); for (int i = 0; i< m_numberOfJoints; i++) { double tmp = m_jointPointers[i]->GetParam(std::string("friction"),0);  r.addDouble(tmp); }
        return true;
    }
    if (key == "hardwareHiStop")
    {
        yarp::os::Bottle& r = val.addList(); for (int i = 0; i< m_numberOfJoints; i++) { double tmp = convertGazeboToUser(i, m_jointPointers[i]->GetUpperLimit(0));  r.addDouble(tmp); }
        return true;
    }
    if (key == "hardwareLowStop")
    {
        yarp::os::Bottle& r = val.addList(); for (int i = 0; i< m_numberOfJoints; i++) { double tmp = convertGazeboToUser(i, m_jointPointers[i]->GetLowerLimit(0));  r.addDouble(tmp); }
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
    if (key == "yarp_jntMaxPos")
    {
        yarp::os::Bottle& r = val.addList(); for (int i = 0; i< m_numberOfJoints; i++) { double tmp_min,tmp_max; getLimits(i,&tmp_min,&tmp_max);  r.addDouble(tmp_max); }
        return true;
    }
    if (key == "yarp_kPWM")
    {
        yarp::os::Bottle& r = val.addList(); for (int i = 0; i< m_numberOfJoints; i++) { r.addDouble(m_kPWM[i]); }
        return true;
    }
    if (key == "yarp_jntMinPos")
    {
        yarp::os::Bottle& r = val.addList(); for (int i = 0; i< m_numberOfJoints; i++) { double tmp_min,tmp_max; getLimits(i,&tmp_min,&tmp_max);  r.addDouble(tmp_min); }
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
    if (key == "VelocityTimeout")
    {   
        yarp::os::Bottle& r = val.addList(); for (int i = 0; i< m_numberOfJoints; i++) { r.addDouble(m_velocity_watchdog[i]->getDuration()); }
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
            double value = bval->get(i).asDouble();
            m_jointPointers[i]->SetDamping(0,value);
        }
        return true;
    }
    if (key == "hardwareFriction")
    {
        for (int i = 0; i < m_numberOfJoints; i++)
        {
            double value = bval->get(i).asDouble();
            m_jointPointers[i]->SetParam("friction",0,value);
        }
        return true;
    }
    if (key == "hardwareEffortLimit")
    {
        for (int i = 0; i < m_numberOfJoints; i++)
        {
            double value = bval->get(i).asDouble();
            m_jointPointers[i]->SetEffortLimit(0,value);
        }
        return true;
    }
    if (key == "hardwareVelocityLimit")
    {
        for (int i = 0; i < m_numberOfJoints; i++)
        {
            double value = bval->get(i).asDouble();
            m_jointPointers[i]->SetVelocityLimit(0,value);
        }
        return true;
    }
    if (key == "hardwareHiStop")
    {
        for (int i = 0; i < m_numberOfJoints; i++)
        {
            double value = bval->get(i).asDouble();
            m_jointPointers[i]->SetUpperLimit(0,convertUserToGazebo(i,value));
        }
        return true;
    }
    if (key == "hardwareLowStop")
    {
        for (int i = 0; i < m_numberOfJoints; i++)
        {
            double value = bval->get(i).asDouble();
            m_jointPointers[i]->SetLowerLimit(0,convertUserToGazebo(i,value));
        }
        return true;
    }
    if (key == "yarp_jntMaxVel")
    {
        for (int i = 0; i < m_numberOfJoints; i++)
        {
            double value = bval->get(i).asDouble();
            setVelLimits(i,0,value);
        }
        return true;
    }
    if (key == "yarp_jntMaxPos")
    {
        for (int i = 0; i < m_numberOfJoints; i++)
        {
            double value = bval->get(i).asDouble();
            double t_max=0, t_min=0;    
            getLimits(i,&t_min,&t_max);
            setLimits(i,t_min,value);
        }
        return true;
    }
    if (key == "yarp_jntMinPos")
    {
        for (int i = 0; i < m_numberOfJoints; i++)
        {
            double value = bval->get(i).asDouble();
            double t_max=0, t_min=0;    
            getLimits(i,&t_min,&t_max);
            setLimits(i,value,t_max);
        }
        return true;
    }
    if (key == "yarp_kPWM")
    {
        for (int i = 0; i < m_numberOfJoints; i++)
        {
            double value = bval->get(i).asDouble();
            m_kPWM[i]=value;
        }
        return true;
    }
    if (key == "SHORTCUT_all_pos_kp")
    {
        for (int i = 0; i < m_numberOfJoints; i++)
        {
            yarp::dev::Pid tmp_pid;
            getPid(i,&tmp_pid);
            tmp_pid.kp = bval->get(i).asDouble();
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
            tmp_pid.kd = bval->get(i).asDouble();
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
            tmp_pid.ki = bval->get(i).asDouble();
            setPid(i,tmp_pid);
        }
        return true;
    }
    if (key == "VelocityTimeout")
    {   
        for (int i = 0; i < m_numberOfJoints; i++)
        {
            double value = bval->get(i).asDouble();
            m_velocity_watchdog[i]->modifyDuration(value);
        }
        return true;
    }
    
    yWarning("setRemoteVariable(): Unknown variable %s", key.c_str());
    return false;
}

