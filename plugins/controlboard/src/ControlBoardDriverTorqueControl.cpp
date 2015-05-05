/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include "ControlBoardDriver.h"


using namespace yarp::dev;

bool GazeboYarpControlBoardDriver::setRefTorque(int j, double t)
{
    if (j >= 0 && j < (int)m_numberOfJoints) {
        m_referenceTorques[j] = t;
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::setRefTorques(const double* t)
{
    if (!t) return false;
    for (unsigned int j = 0; j < m_numberOfJoints; ++j) {
        m_referenceTorques[j] = t[j];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::setTorqueMode()
{
    bool ret = true;
    for (unsigned int j = 0; j < m_numberOfJoints; j++) {
        ret = ret && this->setControlMode(j, VOCAB_CM_TORQUE);
    }
    return ret;
}

bool GazeboYarpControlBoardDriver::getRefTorque(int j, double* t)
{
    if (t && j >= 0 && j < (int)m_numberOfJoints) {
        *t = m_referenceTorques[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getRefTorques(double* t)
{
    if (!t) return false;
    for(unsigned int j = 0; j < m_numberOfJoints; ++j) {
        t[j] = m_referenceTorques[j];
    }
    return true;
}

bool GazeboYarpControlBoardDriver::getTorque(int j, double* t)
{
    if (t && j >= 0 && j < (int)m_numberOfJoints) {
        *t = m_torques[j];
        return true;
    }
    return false;
}

bool GazeboYarpControlBoardDriver::getTorques(double* t)
{
    if (!t) return false;
    for (unsigned int j = 0; j < m_numberOfJoints; ++j) {
        t[j] = m_torques[j];
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
