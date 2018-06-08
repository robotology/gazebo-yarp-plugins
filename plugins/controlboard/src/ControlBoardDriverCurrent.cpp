/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "ControlBoardDriver.h"
#include <yarp/os/LogStream.h>

namespace yarp {
    namespace dev {

        static inline bool NOT_YET_IMPLEMENTED(const char *txt)
        {
            yError() << txt << " is not yet implemented for gazebo_yarp_controlboard";
            return true;
        }

        bool GazeboYarpControlBoardDriver::getCurrentRange(int j, double *min, double *max)
        {
            if (min && max && j >= 0 && static_cast<size_t>(j) < m_numberOfJoints) {
                *min = -m_maxTorques[j] / m_kPWM[j];
                *max = m_maxTorques[j] / m_kPWM[j];
                return true;
            }
            return false;
        }

        bool GazeboYarpControlBoardDriver::getCurrentRanges(double *min, double *max)
        {
            if (!min || !max) return false;
            for (size_t j = 0; j < m_numberOfJoints; ++j) {
                min[j] = -m_maxTorques[j] / m_kPWM[j];
                max[j] = m_maxTorques[j] / m_kPWM[j];
            }
            return true;
        }

        bool GazeboYarpControlBoardDriver::setRefCurrent(int j, double v)
        {
            if (!checkIfTorqueIsValid(v * m_kPWM[j]))
                return false;
            if (j >= 0 && static_cast<size_t>(j) < m_numberOfJoints) {
                m_jntReferenceTorques[j] = v * m_kPWM[j];
                return true;
            }
            return false;
        }

        bool GazeboYarpControlBoardDriver::setRefCurrents(const int n_joint, const int *joints, const double *t)
        {
            if (!joints || !t) return false;
            bool ret = true;
            for (int i = 0; i < n_joint && ret; i++) {
                m_jntReferenceTorques[joints[i]] = t[i] * m_kPWM[joints[i]];
            }
            return ret;
        }

        bool GazeboYarpControlBoardDriver::setRefCurrents(const double* v)
        {
            if (!v) return false;
            for (size_t j = 0; j < m_numberOfJoints; ++j) {
                m_jntReferenceTorques[j] = v[j] * m_kPWM[j];
            }
            return true;
        }

        bool GazeboYarpControlBoardDriver::getCurrent(int j, double* val)
        {
            if (val && j >= 0 && static_cast<size_t>(j) < m_numberOfJoints) {
                *val = m_torques[j] / m_kPWM[j];
                return true;
            }
            return false;
        }

        bool GazeboYarpControlBoardDriver::getCurrents(double *vals)
        {
            if (!vals) return false;
            for (size_t j = 0; j < m_numberOfJoints; ++j) {
                this->getCurrent(j,&vals[j]);
            }
            return true;
        }

        bool GazeboYarpControlBoardDriver::getRefCurrent(int j, double *v)
        {
            if (j >= 0 && static_cast<size_t>(j) < m_numberOfJoints) {
                *v = m_jntReferenceTorques[j] / m_kPWM[j];
                return true;
            }
            return false;
        }

        bool GazeboYarpControlBoardDriver::getRefCurrents(double *v)
        {
            if (!v) return false;
            for (size_t j = 0; j < m_numberOfJoints; ++j) {
                v[j] = m_jntReferenceTorques[j] / m_kPWM[j];
            }
            return true;
        }
        
    }
}
