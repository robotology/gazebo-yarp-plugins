//
//  FakeFakeControlBoardDriverPidControl.c
//  GazeboYarpPlugins
//
//  Created by Francesco Romano on 07/04/14.
//
//

#include "FakeControlBoardDriver.h"
#include <GazeboYarpPlugins/common.h>
#include <gazebo/physics/Joint.hh>
#include <gazebo/transport/Publisher.hh>

namespace yarp {
namespace dev {

bool GazeboYarpFakeControlBoardDriver::setPid (const PidControlTypeEnum& /*pidtype*/, int j, const Pid &pid) {return false;}
bool GazeboYarpFakeControlBoardDriver::setPids (const PidControlTypeEnum& /*pidtype*/, const Pid *pids) {return false;}
bool GazeboYarpFakeControlBoardDriver::setPidReference (const PidControlTypeEnum& /*pidtype*/, int /*j*/, double /*ref*/) { return false; }
bool GazeboYarpFakeControlBoardDriver::setPidReferences (const PidControlTypeEnum& /*pidtype*/, const double */*refs*/) { return false; }
bool GazeboYarpFakeControlBoardDriver::setPidErrorLimit (const PidControlTypeEnum& /*pidtype*/, int /*j*/, double /*limit*/) { return false; }
bool GazeboYarpFakeControlBoardDriver::setPidErrorLimits (const PidControlTypeEnum& /*pidtype*/, const double */*limits*/) { return false; }
bool GazeboYarpFakeControlBoardDriver::getPidError (const PidControlTypeEnum& /*pidtype*/, int j, double *err) {return false;}
bool GazeboYarpFakeControlBoardDriver::getPidErrors (const PidControlTypeEnum& /*pidtype*/, double *errs) {return false;}
bool GazeboYarpFakeControlBoardDriver::getPid (const PidControlTypeEnum& /*pidtype*/, int j, Pid *pid) {return false;}
bool GazeboYarpFakeControlBoardDriver::getPids (const PidControlTypeEnum& /*pidtype*/, Pid * pids) {return false;}
bool GazeboYarpFakeControlBoardDriver::getPidReference (const PidControlTypeEnum& /*pidtype*/, int j, double *ref) {return false;}
bool GazeboYarpFakeControlBoardDriver::getPidReferences (const PidControlTypeEnum& /*pidtype*/, double *refs) {return false;}
bool GazeboYarpFakeControlBoardDriver::getPidErrorLimit (const PidControlTypeEnum& /*pidtype*/, int /*j*/, double */*limit*/) { return false; }
bool GazeboYarpFakeControlBoardDriver::getPidErrorLimits (const PidControlTypeEnum& /*pidtype*/, double */*limits*/) { return false; }
bool GazeboYarpFakeControlBoardDriver::resetPid (const PidControlTypeEnum& /*pidtype*/, int /*j*/) { return false; }
bool GazeboYarpFakeControlBoardDriver::disablePid (const PidControlTypeEnum& /*pidtype*/, int /*j*/) { return false; }
bool GazeboYarpFakeControlBoardDriver::enablePid (const PidControlTypeEnum& /*pidtype*/, int /*j*/) { return false; }
bool GazeboYarpFakeControlBoardDriver::setPidOffset (const PidControlTypeEnum& /*pidtype*/, int /*j*/, double /*v*/) { return false; }
bool GazeboYarpFakeControlBoardDriver::getPidOutput(const PidControlTypeEnum& /*pidtype*/, int j, double *out) { return false; }
bool GazeboYarpFakeControlBoardDriver::getPidOutputs(const PidControlTypeEnum& /*pidtype*/, double *outs) { return false; }
bool GazeboYarpFakeControlBoardDriver::isPidEnabled(const PidControlTypeEnum& pidtype, int j, bool* enabled) { return false; }


}
}
