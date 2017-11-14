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

        bool GazeboYarpControlBoardDriver::setPid(const PidControlTypeEnum& pidtype, int j, const Pid &pid)
        {
            PIDMap::iterator pidType = m_pids.find(pidtype);
            if (pidType == m_pids.end()) {
                yWarning() << "Could not find PID for type " << pidtype;
                return false;
            }

            if (j < 0 || static_cast<size_t>(j) >= pidType->second.size()) {
                yWarning() << "Joint index must be between 0 and " << pidType->second.size();
                return false;
            }

            gazebo::common::PID& currentPID = pidType->second[j];
            // Converting all gains for degrees-based unit to radians-based
            currentPID.SetPGain(convertUserGainToGazeboGain(j, pid.kp));
            currentPID.SetIGain(convertUserGainToGazeboGain(j, pid.ki));
            currentPID.SetDGain(convertUserGainToGazeboGain(j, pid.kd));
            // The output limits are only related to the output, so they don't need to be converted
            currentPID.SetIMax(pid.max_int);
            currentPID.SetIMin(-pid.max_int);
            currentPID.SetCmdMax(pid.max_output);
            currentPID.SetCmdMin(-pid.max_output);

            return true;
        }

        bool GazeboYarpControlBoardDriver::setPids(const PidControlTypeEnum& pidtype, const Pid *pids)
        {
            PIDMap::iterator pidType = m_pids.find(pidtype);
            if (pidType == m_pids.end()) {
                yWarning() << "Could not find PID for type " << pidtype;
                return false;
            }

            bool result = true;
            for (size_t j = 0; j < m_numberOfJoints; ++j) {
                result = result && setPid(pidtype, j, pids[j]);
            }
            return result;
        }

        bool GazeboYarpControlBoardDriver::setPidReference(const PidControlTypeEnum& pidtype, int j, double ref)
        {
            PIDMap::iterator pidType = m_pids.find(pidtype);
            if (pidType == m_pids.end()) {
                yWarning() << "Could not find PID for type " << pidtype;
                return false;
            }

            if (j < 0 || static_cast<size_t>(j) >= pidType->second.size()) {
                yWarning() << "Joint index must be between 0 and " << pidType->second.size();
                return false;
            }
            // Not implemented yet
            return false;
        }

        bool GazeboYarpControlBoardDriver::setPidReferences(const PidControlTypeEnum& pidtype, const double *refs)
        {
            PIDMap::iterator pidType = m_pids.find(pidtype);
            if (pidType == m_pids.end()) {
                yWarning() << "Could not find PID for type " << pidtype;
                return false;
            }
            bool result = true;
            for (size_t j = 0; j < m_numberOfJoints; ++j) {
                result = result && setPidReference(pidtype, j, refs[j]);
            }
            return result;
        }

        bool GazeboYarpControlBoardDriver::setPidErrorLimit(const PidControlTypeEnum& pidtype, int j, double limit)
        {
            PIDMap::iterator pidType = m_pids.find(pidtype);
            if (pidType == m_pids.end()) {
                yWarning() << "Could not find PID for type " << pidtype;
                return false;
            }

            if (j < 0 || static_cast<size_t>(j) >= pidType->second.size()) {
                yWarning() << "Joint index must be between 0 and " << pidType->second.size();
                return false;
            }
            // Not implemented yet
            return false;
        }

        bool GazeboYarpControlBoardDriver::setPidErrorLimits(const PidControlTypeEnum& pidtype, const double *limits)
        {
            PIDMap::iterator pidType = m_pids.find(pidtype);
            if (pidType == m_pids.end()) {
                yWarning() << "Could not find PID for type " << pidtype;
                return false;
            }
            bool result = true;
            for (size_t j = 0; j < m_numberOfJoints; ++j) {
                result = result && setPidErrorLimit(pidtype, j, limits[j]);
            }
            return result;
        }

        bool GazeboYarpControlBoardDriver::getPidError(const PidControlTypeEnum& pidtype, int j, double *err)
        {
            PIDMap::iterator pidType = m_pids.find(pidtype);
            if (pidType == m_pids.end()) {
                yWarning() << "Could not find PID for type " << pidtype;
                return false;
            }

            if (j < 0 || static_cast<size_t>(j) >= pidType->second.size()) {
                yWarning() << "Joint index must be between 0 and " << pidType->second.size();
                return false;
            }

            *err = (m_positions[j] - m_zeroPosition[j]) - m_motReferencePositions[j];
            return true;

        }

        bool GazeboYarpControlBoardDriver::getPidErrors(const PidControlTypeEnum& pidtype, double *errs)
        {
            PIDMap::iterator pidType = m_pids.find(pidtype);
            if (pidType == m_pids.end()) {
                yWarning() << "Could not find PID for type " << pidtype;
                return false;
            }
            bool result = true;
            for (size_t j = 0; j < m_numberOfJoints; ++j) {
                result = result && getPidError(pidtype, j, &errs[j]);
            }
            return result;

        }

        bool GazeboYarpControlBoardDriver::getPidOutput(const PidControlTypeEnum& pidtype, int j, double *out)
        {
            PIDMap::iterator pidType = m_pids.find(pidtype);
            if (pidType == m_pids.end()) {
                yWarning() << "Could not find PID for type " << pidtype;
                return false;
            }

            if (j < 0 || static_cast<size_t>(j) >= pidType->second.size()) {
                yWarning() << "Joint index must be between 0 and " << pidType->second.size();
                return false;
            }

            if (!out) return false;
            out[j] = m_torques[j];

            return true;

        }

        bool GazeboYarpControlBoardDriver::getPidOutputs(const PidControlTypeEnum& pidtype, double *outs)
        {
            PIDMap::iterator pidType = m_pids.find(pidtype);
            if (pidType == m_pids.end()) {
                yWarning() << "Could not find PID for type " << pidtype;
                return false;
            }

            bool result = true;
            for (size_t j = 0; j < m_numberOfJoints; ++j) {
                result = result && getPidOutput(pidtype, j, &outs[j]);
            }
            return result;
        }

        bool GazeboYarpControlBoardDriver::getPid(const PidControlTypeEnum& pidtype, int j, Pid *pid)
        {
            PIDMap::iterator pidType = m_pids.find(pidtype);
            if (pidType == m_pids.end()) {
                yWarning() << "Could not find PID for type " << pidtype;
                return false;
            }

            if (j < 0 || static_cast<size_t>(j) >= pidType->second.size()) {
                yWarning() << "Joint index must be between 0 and " << pidType->second.size();
                return false;
            }
            if (!pid) return false;

            const gazebo::common::PID& currentPID = pidType->second[j];

            // Converting all gains for degrees-based unit to radians-based
            pid->kp = convertGazeboGainToUserGain(j, currentPID.GetPGain());
            pid->ki = convertGazeboGainToUserGain(j, currentPID.GetIGain());
            pid->kd = convertGazeboGainToUserGain(j, currentPID.GetDGain());

            // The output limits are only related to the output, so they don't need to be converted
            pid->max_int = currentPID.GetIMax();
            pid->max_output = currentPID.GetCmdMax();
            return true;

        }

        bool GazeboYarpControlBoardDriver::getPids(const PidControlTypeEnum& pidtype, Pid *pids)
        {
            PIDMap::iterator pidType = m_pids.find(pidtype);
            if (pidType == m_pids.end()) {
                yWarning() << "Could not find PID for type " << pidtype;
                return false;
            }

            bool result = true;
            for (size_t j = 0; j < m_numberOfJoints; ++j) {
                result = result && getPid(pidtype, j, &pids[j]);
            }
            return result;
        }

        bool GazeboYarpControlBoardDriver::getPidReference(const PidControlTypeEnum& pidtype, int j, double *ref)
        {
            PIDMap::iterator pidType = m_pids.find(pidtype);
            if (pidType == m_pids.end()) {
                yWarning() << "Could not find PID for type " << pidtype;
                return false;
            }

            if (j < 0 || static_cast<size_t>(j) >= pidType->second.size()) {
                yWarning() << "Joint index must be between 0 and " << pidType->second.size();
                return false;
            }

            switch (pidtype)
            {
                case VOCAB_PIDTYPE_POSITION:
                    *ref = m_jntReferencePositions[j];
                    break;
                case VOCAB_PIDTYPE_VELOCITY:
                    *ref = m_jntReferenceVelocities[j];
                    break;
                case VOCAB_PIDTYPE_TORQUE:
                case VOCAB_PIDTYPE_CURRENT:
                    *ref = m_jntReferenceTorques[j];
                    break;
            }
            return true;

        }

        bool GazeboYarpControlBoardDriver::getPidReferences(const PidControlTypeEnum& pidtype, double *refs)
        {
            bool result = true;
            for (size_t j = 0; j < m_numberOfJoints; ++j) {
                result = result && getPidReference(pidtype, j, &refs[j]);
            }
            return result;
        }

        bool GazeboYarpControlBoardDriver::getPidErrorLimit(const PidControlTypeEnum& pidtype, int j, double *limit)
        {
            PIDMap::iterator pidType = m_pids.find(pidtype);
            if (pidType == m_pids.end()) {
                yWarning() << "Could not find PID for type " << pidtype;
                return false;
            }

            if (j < 0 || static_cast<size_t>(j) >= pidType->second.size()) {
                yWarning() << "Joint index must be between 0 and " << pidType->second.size();
                return false;
            }

            if (!limit) return false;

            return false; //not implemented
        }

        bool GazeboYarpControlBoardDriver::getPidErrorLimits(const PidControlTypeEnum& pidtype, double *limits)
        {
            PIDMap::iterator pidType = m_pids.find(pidtype);
            if (pidType == m_pids.end()) {
                yWarning() << "Could not find PID for type " << pidtype;
                return false;
            }

            bool result = true;
            for (size_t j = 0; j < m_numberOfJoints; ++j) {
                result = result && getPidErrorLimit(pidtype, j, &limits[j]);
            }
            return result;
        }

        bool GazeboYarpControlBoardDriver::resetPid(const PidControlTypeEnum& pidtype, int j)
        {
            PIDMap::iterator pidType = m_pids.find(pidtype);
            if (pidType == m_pids.end()) {
                yWarning() << "Could not find PID for type " << pidtype;
                return false;
            }

            if (j < 0 || static_cast<size_t>(j) >= pidType->second.size()) {
                yWarning() << "Joint index must be between 0 and " << pidType->second.size();
                return false;
            }

            return false; //not implemented
        }

        bool GazeboYarpControlBoardDriver::disablePid(const PidControlTypeEnum& pidtype, int j)
        {
            PIDMap::iterator pidType = m_pids.find(pidtype);
            if (pidType == m_pids.end()) {
                yWarning() << "Could not find PID for type " << pidtype;
                return false;
            }

            if (j < 0 || static_cast<size_t>(j) >= pidType->second.size()) {
                yWarning() << "Joint index must be between 0 and " << pidType->second.size();
                return false;
            }

            return false; //not implemented
        }

        bool GazeboYarpControlBoardDriver::enablePid(const PidControlTypeEnum& pidtype, int j)
        {
            PIDMap::iterator pidType = m_pids.find(pidtype);
            if (pidType == m_pids.end()) {
                yWarning() << "Could not find PID for type " << pidtype;
                return false;
            }

            if (j < 0 || static_cast<size_t>(j) >= pidType->second.size()) {
                yWarning() << "Joint index must be between 0 and " << pidType->second.size();
                return false;
            }

            return false; //not implemented
        }

        bool GazeboYarpControlBoardDriver::setPidOffset(const PidControlTypeEnum& pidtype, int j, double v)
        {
            PIDMap::iterator pidType = m_pids.find(pidtype);
            if (pidType == m_pids.end()) {
                yWarning() << "Could not find PID for type " << pidtype;
                return false;
            }

            if (j < 0 || static_cast<size_t>(j) >= pidType->second.size()) {
                yWarning() << "Joint index must be between 0 and " << pidType->second.size();
                return false;
            }

            return false; //not implemented
        }

        bool GazeboYarpControlBoardDriver::isPidEnabled(const PidControlTypeEnum& pidtype, int j, bool* enabled)
        {
            PIDMap::iterator pidType = m_pids.find(pidtype);
            if (pidType == m_pids.end()) {
                yWarning() << "Could not find PID for type " << pidtype;
                return false;
            }
            
            if (j < 0 || static_cast<size_t>(j) >= pidType->second.size()) {
                yWarning() << "Joint index must be between 0 and " << pidType->second.size();
                return false;
            }
            
            return false; //not implemented
        }
        
    }
}
