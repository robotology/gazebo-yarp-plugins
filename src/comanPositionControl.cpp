#include "coman.h"


using namespace yarp::dev;


/**
 * This is asyncronous, but do we care?
 */
bool coman::positionMove(int j, double ref) //WORKS
{
    if (j<_robot_number_of_joints) {
        ref_pos[j] = ref; //we will use this ref_pos in the next simulation onUpdate call to ask gazebo to set PIDs ref_pos to this value
    }
    return true;
}

bool coman::stop(int j) //WORKS
{
    ref_pos[j]=pos[j];
    return true;
}

bool coman::stop() //WORKS
{
    ref_pos=pos;
    return true;
}

bool coman::positionMove(const double *refs) //WORKS
{
    for (int i=0; i<_robot_number_of_joints; ++i) {
        ref_pos[i] = refs[i];
    }
    return true;
}

bool coman::getAxes(int *ax) // WORKS
{
    *ax = _robot_number_of_joints;
    return true;
}

bool coman::setRefSpeed(int j, double sp) //WORKS
{
    if (j<_robot_number_of_joints) {
        ref_speed[j] = sp;
    }
    return true;
}

bool coman::getRefSpeed(int j, double *ref) //WORKS
{
    if (j<_robot_number_of_joints) {
        (*ref) = ref_speed[j];
    }
    return true;
}

bool coman::getRefSpeeds(double *spds) //WORKS
{
    for (int i=0; i<_robot_number_of_joints; ++i) {
        spds[i] = ref_speed[i];
    }
    return true;
}



bool coman::relativeMove(int j, double delta) //NOT TESTED
{
    if (j<_robot_number_of_joints) {
        ref_pos[j] =pos[j] + delta; //TODO check if this is ok or ref_pos=ref_pos+delta!!!
    }
    return true;
}

bool coman::relativeMove(const double *deltas) //NOT TESTED
{
    for (int i=0; i<_robot_number_of_joints; ++i) {
        ref_pos[i] = pos[i]+ deltas[i]; //TODO check if this is ok or ref_pos=ref_pos+delta!!!
    }
    return true;
}

bool coman::checkMotionDone(int j, bool *flag) //NOT TESTED
{
    *flag=motion_done[j];
    return true;
}

bool coman::checkMotionDone(bool *flag) //NOT TESTED
{
    bool temp_flag=true;
    //*flag=true;
    for(int j=0; j<_robot_number_of_joints; ++j)
    {
        //*flag&&motion_done[j]; //It's compiler job to make code unreadable and optimized, not programmer's
        temp_flag=temp_flag && motion_done[j];
    }
    *flag=temp_flag;
    return true;
}

bool coman::setPositionMode() //NOT TESTED
{
    for(int j=0; j<_robot_number_of_joints; j++)
    {
        this->setPositionMode(j);
    }
}

bool coman::setRefSpeeds(const double *spds) //NOT TESTED
{
    for (int i=0; i<_robot_number_of_joints; ++i) {
        ref_speed[i] = spds[i];
    }
    return true;
}




bool coman::setRefAcceleration(int j, double acc) //NOT IMPLEMENTED
{
    if (j<_robot_number_of_joints) {
        ref_acc[j] = acc;
    }
    return true;
}

bool coman::setRefAccelerations(const double *accs) //NOT IMPLEMENTED
{
    for (int i=0; i<_robot_number_of_joints; ++i) {
        ref_acc[i] = accs[i];
    }
    return true;
}

bool coman::getRefAcceleration(int j, double *acc) //NOT IMPLEMENTED
{
    if (j<_robot_number_of_joints) {
        (*acc) = ref_acc[j];
    }
    return true;
}

bool coman::getRefAccelerations(double *accs) //NOT IMPLEMENTED
{
    for (int i=0; i<_robot_number_of_joints; ++i) {
        accs[i] = ref_acc[i];
    }
    return true;
}