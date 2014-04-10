/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <iostream>
#include "gazebo_yarp_plugins/ControlBoardDriver.h"

namespace yarp {
    namespace dev {
        
        bool GazeboYarpControlBoardDriver::setOutput(int j, double v)
        {
            if (j >= 0 && j < (int)numberOfJoints)
            {
                referenceTorque[j] = v;
                return true;
            }
            return false;
        }
        bool GazeboYarpControlBoardDriver::setOutputs(const double *v)
        {
            if (!v) return false;
            for (unsigned int j = 0; j < numberOfJoints; ++j)
            {
                referenceTorque[j] = v[j];
            }
            return true;
        }
        
        bool GazeboYarpControlBoardDriver::getOutput(int j, double *v)
        {
            if (v && j >= 0 && j < (int)numberOfJoints) {
                *v = torque[j];
                return true;
            }
            return false;
        }
        
        bool GazeboYarpControlBoardDriver::getOutputs(double *v)
        {
            std::cout << "get outputs - gazebo\n";
            if (!v) return false;
            for(unsigned int j = 0; j < numberOfJoints; ++j) {
                v[j] = torque[j];
            }
            return true;
        }
        
        bool GazeboYarpControlBoardDriver::setOpenLoopMode()
        {
            for(unsigned int j = 0; j < numberOfJoints; j++)
            {
                this->setOpenLoopMode(j);
            }
            return true;
        }
        
    }
}
