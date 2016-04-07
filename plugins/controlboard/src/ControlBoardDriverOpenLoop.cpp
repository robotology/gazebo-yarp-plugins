/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include <iostream>
#include "ControlBoardDriver.h"

namespace yarp {
    namespace dev {

        bool GazeboYarpControlBoardDriver::setRefOutput(int j, double v)
        {
            if (j >= 0 && j < (int)m_numberOfJoints) {
                m_jntReferenceTorques[j] = v*m_kPWM[j];
                return true;
            }
            return false;
        }

        bool GazeboYarpControlBoardDriver::setRefOutputs(const double* v)
        {
            if (!v) return false;
            for (unsigned int j = 0; j < m_numberOfJoints; ++j) {
                m_jntReferenceTorques[j] = v[j]*m_kPWM[j];
            }
            return true;
        }

        bool GazeboYarpControlBoardDriver::getOutput(int j, double *v)
        {
            if (v && j >= 0 && j < (int)m_numberOfJoints) {
                *v = m_torques[j];
                return true;
            }
            return false;
        }

        bool GazeboYarpControlBoardDriver::getOutputs(double *v)
        {
            if (!v) return false;
            for (unsigned int j = 0; j < m_numberOfJoints; ++j) {
                v[j] = m_torques[j];
            }
            return true;
        }

        bool GazeboYarpControlBoardDriver::getRefOutput(int j, double *v)
        {
            if (v && j >= 0 && j < (int)m_numberOfJoints) {
                *v = m_jntReferenceTorques[j];
                return true;
            }
            return false;
        }

        bool GazeboYarpControlBoardDriver::getRefOutputs(double *v)
        {
            if (!v) return false;
            for (unsigned int j = 0; j < m_numberOfJoints; ++j) {
                v[j] = m_jntReferenceTorques[j];
            }
            return true;
        }

        bool GazeboYarpControlBoardDriver::setOpenLoopMode()
        {
            bool ret = true;
            for (unsigned int j = 0; j < m_numberOfJoints; j++) {
                ret = ret && this->setControlMode(j, VOCAB_CM_OPENLOOP);
            }
            return ret;
        }

    }
}
