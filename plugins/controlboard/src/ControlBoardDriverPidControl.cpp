//
//  ControlBoardDriverPidControl.c
//  GazeboYarpPlugins
//
//  Created by Francesco Romano on 07/04/14.
//
//

#include "ControlBoardDriver.h"
#include <GazeboYarpPlugins/common.h>
#include <gazebo/physics/Joint.hh>
#include <gazebo/transport/Publisher.hh>

namespace yarp {
    namespace dev {

        bool GazeboYarpControlBoardDriver::setPid (int j, const Pid &pid)
        {
            // Converting all gains for degrees-based unit to radians-based
            m_positionPIDs[j].p = convertUserGainToGazeboGain(j, pid.kp);
            m_positionPIDs[j].i = convertUserGainToGazeboGain(j, pid.ki);
            m_positionPIDs[j].d = convertUserGainToGazeboGain(j, pid.kd);
            // The output limits are only related to the output, so they don't need to be converted
            m_positionPIDs[j].maxInt = pid.max_int;
            m_positionPIDs[j].maxOut = pid.max_output;

            gazebo::msgs::JointCmd j_cmd;
            j_cmd.set_name(m_jointPointers[j]->GetScopedName());
            j_cmd.mutable_position()->clear_target();       // This function marks the target field as unused (and it sets internal value to zero)
            j_cmd.mutable_position()->set_p_gain(m_positionPIDs[j].p);
            j_cmd.mutable_position()->set_i_gain(m_positionPIDs[j].i );
            j_cmd.mutable_position()->set_d_gain( m_positionPIDs[j].d);
            j_cmd.mutable_position()->set_i_max(m_positionPIDs[j].maxInt);
            j_cmd.mutable_position()->set_i_min(-m_positionPIDs[j].maxInt);
            j_cmd.mutable_position()->set_limit(m_positionPIDs[j].maxOut);

            m_jointCommandPublisher->WaitForConnection();
            m_jointCommandPublisher->Publish(j_cmd);
            return true;
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

        bool GazeboYarpControlBoardDriver::getError (int j, double *err)
        {
            *err=(m_positions[j]-m_zeroPosition[j])-m_motReferencePositions[j];
            return true;
        }

        bool GazeboYarpControlBoardDriver::getErrors (double *errs)
        {
            for(int j=0; j< m_numberOfJoints; j++)
                getError(j, &errs[j]);
            return true;
        }

        bool GazeboYarpControlBoardDriver::getPid (int j, Pid *pid)
        {
            // Converting all gains for degrees-based unit to radians-based
            pid->kp = convertGazeboGainToUserGain(j, m_positionPIDs[j].p);
            pid->ki = convertGazeboGainToUserGain(j, m_positionPIDs[j].i);
            pid->kd = convertGazeboGainToUserGain(j, m_positionPIDs[j].d);

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
            *ref = m_jntReferencePositions[j];
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
