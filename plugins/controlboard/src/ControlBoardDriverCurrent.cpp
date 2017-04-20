/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include <iostream>
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
            return NOT_YET_IMPLEMENTED("getCurrentRange");
        }

        bool GazeboYarpControlBoardDriver::getCurrentRanges(double *min, double *max)
        {
            return NOT_YET_IMPLEMENTED("getCurrentRanges");
        }

        bool GazeboYarpControlBoardDriver::setRefCurrent(int j, double v)
        {
            if (j >= 0 && j < (int)m_numberOfJoints) {
                m_jntReferenceTorques[j] = v*m_kPWM[j];
                return true;
            }
            return false;
        }

        bool GazeboYarpControlBoardDriver::setRefCurrents(const int n_joint, const int *joints, const double *t)
        {
            if (!joints || !t) return false;
            bool ret = true;
            for (int i = 0; i < n_joint && ret; i++)
            {
                ret = setRefCurrent(joints[i], t[i]);
            }
            return ret;
        }

        bool GazeboYarpControlBoardDriver::setRefCurrents(const double* v)
        {
            if (!v) return false;
            for (unsigned int j = 0; j < m_numberOfJoints; ++j) {
                m_jntReferenceTorques[j] = v[j]*m_kPWM[j];
            }
            return true;
        }

        /*
        //Already implemented by another interface
        bool GazeboYarpControlBoardDriver::getCurrent(int j, double *v)
        {
            if (val && j >= 0 && j < (int)m_numberOfJoints) {
                *val = amp[j];
                return true;
            }
            return false;
        }
        */

        /*
        //Already implemented by another interface
        bool GazeboYarpControlBoardDriver::getCurrents(double *v)
        {
            if (!vals) return false;
            for (unsigned int i=0; i<m_numberOfJoints; i++) {
                vals[i] = amp[i];
            }
            return true;
        }
        */


        bool GazeboYarpControlBoardDriver::getRefCurrent(int j, double *v)
        {
            if (v && j >= 0 && j < (int)m_numberOfJoints) {
                *v = m_jntReferenceTorques[j];
                return true;
            }
            return false;
        }

        bool GazeboYarpControlBoardDriver::getRefCurrents(double *v)
        {
            if (!v) return false;
            for (unsigned int j = 0; j < m_numberOfJoints; ++j) {
                v[j] = m_jntReferenceTorques[j];
            }
            return true;
        }

        bool GazeboYarpControlBoardDriver::setCurrentPid(int j, const Pid &pid)
        {
            return NOT_YET_IMPLEMENTED("setCurrentPidRaw");
        }

        bool GazeboYarpControlBoardDriver::setCurrentPids(const Pid *pids)
        {
            return NOT_YET_IMPLEMENTED("setCurrentPidsRaw");
        }

        bool GazeboYarpControlBoardDriver::getCurrentError(int j, double *err)
        {
            return NOT_YET_IMPLEMENTED("getCurrentErrorRaw");
        }

        bool GazeboYarpControlBoardDriver::getCurrentErrors(double *errs)
        {
            return NOT_YET_IMPLEMENTED("getCurrentErrorsRaw");
        }

        bool GazeboYarpControlBoardDriver::getCurrentPidOutput(int j, double *out)
        {
            return NOT_YET_IMPLEMENTED("getCurrentPidOutputRaw");
        }

        bool GazeboYarpControlBoardDriver::getCurrentPidOutputs(double *outs)
        {
            return NOT_YET_IMPLEMENTED("getCurrentPidOutputsRaw");
        }

        bool GazeboYarpControlBoardDriver::getCurrentPid(int j, Pid *pid)
        {
            return NOT_YET_IMPLEMENTED("getCurrentPidRaw");
        }

        bool GazeboYarpControlBoardDriver::getCurrentPids(Pid *pids)
        {
            return NOT_YET_IMPLEMENTED("getCurrentPidsRaw");
        }

        bool GazeboYarpControlBoardDriver::resetCurrentPid(int j)
        {
            return NOT_YET_IMPLEMENTED("resetCurrentPidRaw");
        }

        bool GazeboYarpControlBoardDriver::disableCurrentPid(int j)
        {
            return NOT_YET_IMPLEMENTED("disableCurrentPidRaw");
        }

        bool GazeboYarpControlBoardDriver::enableCurrentPid(int j)
        {
            return NOT_YET_IMPLEMENTED("enableCurrentPidRaw");
        }

    }
}
