/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "FakeControlBoardDriver.h"
#include <gazebo/physics/Joint.hh>

using namespace yarp::dev;

bool GazeboYarpFakeControlBoardDriver::getRemoteVariablesList(yarp::os::Bottle* listOfKeys)
{
    listOfKeys->clear();
    return true;
}

bool GazeboYarpFakeControlBoardDriver::getRemoteVariable(std::string key, yarp::os::Bottle& val)
{
    val.clear();
    yWarning("getRemoteVariable(): Unknown variable %s", key.c_str());
    return false;
}

bool GazeboYarpFakeControlBoardDriver::setRemoteVariable(std::string key, const yarp::os::Bottle& val)
{
    yWarning("setRemoteVariable(): Unknown variable %s", key.c_str());
    return false;
}

