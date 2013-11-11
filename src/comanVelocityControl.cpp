/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include "coman.h"

using namespace yarp::dev;


bool GazeboYarpControlBoardDriver::setVelocityMode() //NOT TESTED
{
    for(unsigned int j=0; j<_robot_number_of_joints; j++)
    {
        this->setVelocityMode(j);
    }
    return true;
}

bool GazeboYarpControlBoardDriver::velocityMove(int j, double sp) //NOT TESTED
{
    if (j<_robot_number_of_joints) 
    {
        vel[j] = sp;
    }
    return true;
}

bool GazeboYarpControlBoardDriver::velocityMove(const double *sp) //NOT TESTED
{
    for (unsigned int i=0; i<_robot_number_of_joints; ++i) {
        vel[i] = sp[i];
    }
    return true;
}
