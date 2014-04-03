/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include "gazebo_yarp_plugins/ControlBoardDriver.h"

namespace yarp {
    namespace dev {
        
        bool GazeboYarpControlBoardDriver::setOutput(int j, double v)
        {
            return false;
        }
        bool GazeboYarpControlBoardDriver::setOutputs(const double *v)
        {
            return false;
        }
        
        bool GazeboYarpControlBoardDriver::getOutput(int j, double *v)
        {
            return false;
        }
        
        bool GazeboYarpControlBoardDriver::getOutputs(double *v)
        {
            return false;
        }
        
        bool GazeboYarpControlBoardDriver::setOpenLoopMode()
        {
            return false;
        }
        
    }
}