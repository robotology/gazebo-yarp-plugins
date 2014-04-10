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
            if (j >= 0 && j < (int)_controlboard_number_of_joints)
            {
                ref_torque[j] = v;
                return true;
            }
            return false;
        }
        bool GazeboYarpControlBoardDriver::setOutputs(const double *v)
        {
            if (!v) return false;
            for (unsigned int j = 0; j < _controlboard_number_of_joints; ++j)
            {
                ref_torque[j] = v[j];
            }
            return true;
        }
        
        bool GazeboYarpControlBoardDriver::getOutput(int j, double *v)
        {
            if (v && j >= 0 && j < (int)_controlboard_number_of_joints) {
                *v = torque[j];
                return true;
            }
            return false;
        }
        
        bool GazeboYarpControlBoardDriver::getOutputs(double *v)
        {
            std::cout << "get outputs - gazebo\n";
            if (!v) return false;
            for(unsigned int j = 0; j < _controlboard_number_of_joints; ++j) {
                v[j] = torque[j];
            }
            return true;
        }
        
        bool GazeboYarpControlBoardDriver::setOpenLoopMode()
        {
            for(unsigned int j = 0; j < _controlboard_number_of_joints; j++)
            {
                this->setOpenLoopMode(j);
            }
            return true;
        }
        
    }
}
