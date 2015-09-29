//
//  ControlBoardDriverPidControl.c
//  GazeboYarpPlugins
//
//  Created by Francesco Romano on 07/04/14.
//
//

#include "ControlBoardDriver.h"
#include <GazeboYarpPlugins/common.h>

namespace yarp {
    namespace dev {

        bool GazeboYarpControlBoardDriver::setPid (int j, const Pid &pid)
        {
            // Converting all gains for degrees-based unit to radians-based
            m_positionPIDs[j].p = GazeboYarpPlugins::convertDegreesGainsToRadiansGains(pid.kp);
            m_positionPIDs[j].i = GazeboYarpPlugins::convertDegreesGainsToRadiansGains(pid.ki);
            m_positionPIDs[j].d = GazeboYarpPlugins::convertDegreesGainsToRadiansGains(pid.kd);
            // The output limits are only related to the output, so they don't need to be converted
            m_positionPIDs[j].maxInt = pid.max_int;
            m_positionPIDs[j].maxOut = pid.max_output;
        }

        bool GazeboYarpControlBoardDriver::setPids (const Pid *pids)
        {
            for(int j=0; j< m_numberOfJoints; j++)
                setPid(j, (pids[j]));
            return true;
        }

        bool GazeboYarpControlBoardDriver::setReference (int /*j*/, double /*ref*/) { return false; }
        bool GazeboYarpControlBoardDriver::setReferences (const double */*refs*/) { return false; }
        bool GazeboYarpControlBoardDriver::setErrorLimit (int /*j*/, double /*limit*/) { return false; }
        bool GazeboYarpControlBoardDriver::setErrorLimits (const double */*limits*/) { return false; }
        bool GazeboYarpControlBoardDriver::getError (int /*j*/, double */*err*/) { return false; }
        bool GazeboYarpControlBoardDriver::getErrors (double */*errs*/) { return false; }

        bool GazeboYarpControlBoardDriver::getPid (int j, Pid *pid)
        {
            // Converting all gains for degrees-based unit to radians-based
            pid->kp = GazeboYarpPlugins::convertRadiansGainsToDegreesGains(m_positionPIDs[j].p);
            pid->ki = GazeboYarpPlugins::convertRadiansGainsToDegreesGains(m_positionPIDs[j].i);
            pid->kd = GazeboYarpPlugins::convertRadiansGainsToDegreesGains(m_positionPIDs[j].d);

            // The output limits are only related to the output, so they don't need to be converted
            pid->max_int = m_positionPIDs[j].maxInt;
            pid->max_output = m_positionPIDs[j].maxOut;
            return true;
        }

        bool GazeboYarpControlBoardDriver::getPids (Pid * pids)
        {
            for(int j=0; j< m_numberOfJoints; j++)
                getPid(j, &(pids[j]));
            return true;
        }

        bool GazeboYarpControlBoardDriver::getReference (int j, double *ref)
        {
            *ref = m_referencePositions[j];
            return true;
        }

        bool GazeboYarpControlBoardDriver::getReferences (double *refs)
        {
            for(int j=0; j< m_numberOfJoints; j++)
                getReference(j, &refs[j]);
            return true;
        }

        bool GazeboYarpControlBoardDriver::getErrorLimit (int /*j*/, double */*limit*/) { return false; }
        bool GazeboYarpControlBoardDriver::getErrorLimits (double */*limits*/) { return false; }
        bool GazeboYarpControlBoardDriver::resetPid (int /*j*/) { return false; }
        bool GazeboYarpControlBoardDriver::disablePid (int /*j*/) { return false; }
        bool GazeboYarpControlBoardDriver::enablePid (int /*j*/) { return false; }
        bool GazeboYarpControlBoardDriver::setOffset (int /*j*/, double /*v*/) { return false; }

    }
}
