#include "coman.h"

using namespace yarp::dev;

bool coman::getTorqueRange(int j, double *min, double *max){return false;} //NOT IMPLEMENTED
bool coman::getTorqueRanges(double *min, double *max){return false;} //NOT IMPLEMENTED
bool coman::setTorquePids(const Pid *pids){return false;} //NOT IMPLEMENTED
bool coman::setTorqueErrorLimit(int j, double limit){return false;} //NOT IMPLEMENTED
bool coman::setTorqueErrorLimits(const double *limits){return false;} //NOT IMPLEMENTED
bool coman::getTorqueError(int j, double *err){return false;} //NOT IMPLEMENTED
bool coman::getTorqueErrors(double *errs){return false;} //NOT IMPLEMENTED
bool coman::getTorquePidOutput(int j, double *out){return false;} //NOT IMPLEMENTED
bool coman::getTorquePidOutputs(double *outs){return false;} //NOT IMPLEMENTED
bool coman::getTorquePid(int j, Pid *pid){return false;} //NOT IMPLEMENTED
bool coman::getTorquePids(Pid *pids){return false;} //NOT IMPLEMENTED
bool coman::getTorqueErrorLimit(int j, double *limit){return false;} //NOT IMPLEMENTED
bool coman::getTorqueErrorLimits(double *limits){return false;} //NOT IMPLEMENTED
bool coman::resetTorquePid(int j){return false;} //NOT IMPLEMENTED
bool coman::disableTorquePid(int j){return false;} //NOT IMPLEMENTED
bool coman::enableTorquePid(int j){return false;} //NOT IMPLEMENTED
bool coman::setTorqueOffset(int j, double v){return false;} //NOT IMPLEMENTED
bool coman::getBemfParam(int j, double *bemf){return false;} //NOT IMPLEMENTED
bool coman::setBemfParam(int j, double bemf){return false;} //NOT IMPLEMENTED
bool coman::setTorquePid(int j, const Pid &pid){return false;} //NOT IMPLEMENTED


bool coman::setRefTorque(int j, double t) //NOT TESTED
{
    std::cout<<std::endl<<"Joint"<<j<<" trq: "<<t<<std::endl<<std::endl;
    if (j<_robot_number_of_joints)
    {
        ref_torque[j] = t;
    }
    return true;
}

bool coman::setRefTorques(const double *t) //NOT TESTED
{
    for (unsigned int i=0; i<_robot_number_of_joints; ++i)
        setRefTorque(i, t[i]);
    return true;
}


bool coman::setTorqueMode() //NOT TESTED
{
    for(int j=0; j<_robot_number_of_joints; j++)
    {
        this->setTorqueMode(j);
    }
}

bool coman::getRefTorque(int j, double *t)
{
    if (j<_robot_number_of_joints) {
        t[j] = ref_torque[j];
    }
    return true;
} //NOT TESTED


bool coman::getRefTorques(double *t)
{
    for(unsigned int i = 0; i < _robot_number_of_joints; ++i)
        getRefTorque(i, t);
    return true;
} //NOT TESTED




bool coman::getTorque(int j, double *t)
{
    if (j<_robot_number_of_joints) {
        t[j] = torque[j];
    }
    return true;
} //NOT TESTED
bool coman::getTorques(double *t)
{
    for(unsigned int i = 0; i < _robot_number_of_joints; ++i)
        getTorque(i, t);
    return true;
} //NOT TESTED