#include "coman.h"

using namespace yarp::dev;


bool coman::setVelocityMode() //NOT TESTED
{
    for(unsigned int j=0; j<_robot_number_of_joints; j++)
    {
        this->setVelocityMode(j);
    }
    return true;
}

bool coman::velocityMove(int j, double sp) //NOT TESTED
{
    if (j<_robot_number_of_joints) 
    {
        vel[j] = sp;
    }
    return true;
}

bool coman::velocityMove(const double *sp) //NOT TESTED
{
    for (unsigned int i=0; i<_robot_number_of_joints; ++i) {
        vel[i] = sp[i];
    }
    return true;
}
