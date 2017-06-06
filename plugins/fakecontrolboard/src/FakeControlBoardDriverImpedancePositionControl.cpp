/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "FakeControlBoardDriver.h"

using namespace yarp::dev;


bool GazeboYarpFakeControlBoardDriver::getImpedance(int j, double *stiffness, double *damping)  {return false;}
bool GazeboYarpFakeControlBoardDriver::setImpedance(int j, double stiffness, double damping) {return false;}
bool GazeboYarpFakeControlBoardDriver::setImpedanceOffset(int j, double offset)  {return false;}
bool GazeboYarpFakeControlBoardDriver::getImpedanceOffset(int j, double* offset) {return false;}
bool GazeboYarpFakeControlBoardDriver::getCurrentImpedanceLimit(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp) {return false;}
