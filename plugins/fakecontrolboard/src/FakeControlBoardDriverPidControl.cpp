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

bool GazeboYarpFakeControlBoardDriver::setPid (int j, const Pid &pid) {return false;}
bool GazeboYarpFakeControlBoardDriver::setPids (const Pid *pids) {return false;}
bool GazeboYarpFakeControlBoardDriver::setReference (int /*j*/, double /*ref*/) { return false; }
bool GazeboYarpFakeControlBoardDriver::setReferences (const double */*refs*/) { return false; }
bool GazeboYarpFakeControlBoardDriver::setErrorLimit (int /*j*/, double /*limit*/) { return false; }
bool GazeboYarpFakeControlBoardDriver::setErrorLimits (const double */*limits*/) { return false; }
bool GazeboYarpFakeControlBoardDriver::getError (int j, double *err) {return false;}
bool GazeboYarpFakeControlBoardDriver::getErrors (double *errs) {return false;}
bool GazeboYarpFakeControlBoardDriver::getPid (int j, Pid *pid) {return false;}
bool GazeboYarpFakeControlBoardDriver::getPids (Pid * pids) {return false;}
bool GazeboYarpFakeControlBoardDriver::getReference (int j, double *ref) {return false;}
bool GazeboYarpFakeControlBoardDriver::getReferences (double *refs) {return false;}
bool GazeboYarpFakeControlBoardDriver::getErrorLimit (int /*j*/, double */*limit*/) { return false; }
bool GazeboYarpFakeControlBoardDriver::getErrorLimits (double */*limits*/) { return false; }
bool GazeboYarpFakeControlBoardDriver::resetPid (int /*j*/) { return false; }
bool GazeboYarpFakeControlBoardDriver::disablePid (int /*j*/) { return false; }
bool GazeboYarpFakeControlBoardDriver::enablePid (int /*j*/) { return false; }
bool GazeboYarpFakeControlBoardDriver::setOffset (int /*j*/, double /*v*/) { return false; }

}
}
