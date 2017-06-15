/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include <iostream>
#include "ControlBoardDriver.h"

namespace yarp {
    namespace dev {

        bool GazeboYarpControlBoardDriver::getNumberOfMotors(int *ax)
        {
            if (!ax) return false;
            *ax = m_numberOfJoints;
            return true;
        }

        bool GazeboYarpControlBoardDriver::setRefDutyCycle(int j, double v)
        {
            if (j >= 0 && static_cast<size_t>(j) < m_numberOfJoints) {
                m_jntReferenceTorques[j] = v*m_kPWM[j];
                return true;
            }
            return false;
        }

        bool GazeboYarpControlBoardDriver::setRefDutyCycles(const double* v)
        {
            if (!v) return false;
            for (size_t j = 0; j < m_numberOfJoints; ++j) {
                m_jntReferenceTorques[j] = v[j]*m_kPWM[j];
            }
            return true;
        }

        bool GazeboYarpControlBoardDriver::getDutyCycle(int j, double *v)
        {
            if (v && j >= 0 && static_cast<size_t>(j) < m_numberOfJoints) {
                *v = m_torques[j];
                return true;
            }
            return false;
        }

        bool GazeboYarpControlBoardDriver::getDutyCycles(double *v)
        {
            if (!v) return false;
            for (size_t j = 0; j < m_numberOfJoints; ++j) {
                v[j] = m_torques[j];
            }
            return true;
        }

        bool GazeboYarpControlBoardDriver::getRefDutyCycle(int j, double *v)
        {
            if (v && j >= 0 && static_cast<size_t>(j) < m_numberOfJoints) {
                *v = m_jntReferenceTorques[j];
                return true;
            }
            return false;
        }

        bool GazeboYarpControlBoardDriver::getRefDutyCycles(double *v)
        {
            if (!v) return false;
            for (size_t j = 0; j < m_numberOfJoints; ++j) {
                v[j] = m_jntReferenceTorques[j];
            }
            return true;
        }
    }
}
