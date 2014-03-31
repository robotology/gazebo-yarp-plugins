/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include "gazebo_yarp_plugins/ControlBoardDriver.h"


using namespace yarp::dev;

bool GazeboYarpControlBoardDriver::setRefTorque(int j, double t) //NOT TESTED
{
//    std::cout << std::endl << "Joint" << j << " trq: " << t << std::endl;
    if (j >= 0 && j < (int)_controlboard_number_of_joints)
    {
        ref_torque[j] = t;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::setRefTorques(const double *t) //NOT TESTED
{
    if (!t) return false;
    for (unsigned int j = 0; j < _controlboard_number_of_joints; ++j)
    {
//        std::cout << std::endl << "Joint" << j << " trq: " << t[j] << std::endl;
        ref_torque[j] = t[j];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::setTorqueMode() //NOT TESTED
{
//    std::cout<<std::endl<<" set torque mode "<< std::endl;
    for(unsigned int j = 0; j < _controlboard_number_of_joints; j++)
    {
        this->setTorqueMode(j);
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getRefTorque(int j, double *t) //NOT TESTED
{
    if (t && j >= 0 && j < (int)_controlboard_number_of_joints) {
        *t = ref_torque[j];
        return true;
    }
    return false;
} 

bool GazeboYarpControlBoardDriver::getRefTorques(double *t) //NOT TESTED
{
    if (!t) return false;
    for(unsigned int j = 0; j < _controlboard_number_of_joints; ++j) {
        t[j] = ref_torque[j];
    }
    return true;
} 

bool GazeboYarpControlBoardDriver::getTorque(int j, double *t) //NOT TESTED
{
    if (t && j >= 0 && j < (int)_controlboard_number_of_joints) {
        *t = torque[j];
        return true;
    }
    return false;
} 

bool GazeboYarpControlBoardDriver::getTorques(double *t) //NOT TESTED
{
    if (!t) return false;
    for(unsigned int j = 0; j < _controlboard_number_of_joints; ++j) {
        t[j] = torque[j];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getTorqueRange(int, double*, double *){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::getTorqueRanges(double *, double *){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::setTorquePids(const Pid *){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::setTorqueErrorLimit(int , double ){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::setTorqueErrorLimits(const double *){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::getTorqueError(int , double *){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::getTorqueErrors(double *){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::getTorquePidOutput(int , double *){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::getTorquePidOutputs(double *){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::getTorquePid(int , Pid *){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::getTorquePids(Pid *){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::getTorqueErrorLimit(int , double *){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::getTorqueErrorLimits(double *){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::resetTorquePid(int ){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::disableTorquePid(int ){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::enableTorquePid(int ){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::setTorqueOffset(int , double ){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::getBemfParam(int , double *){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::setBemfParam(int , double ){return false;} //NOT IMPLEMENTED
bool GazeboYarpControlBoardDriver::setTorquePid(int , const Pid &){return false;} //NOT IMPLEMENTED

