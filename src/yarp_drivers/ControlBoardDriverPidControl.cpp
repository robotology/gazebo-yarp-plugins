//
//  ControlBoardDriverPidControl.c
//  GazeboYarpPlugins
//
//  Created by Francesco Romano on 07/04/14.
//
//

#include "ControlBoardDriver.h"

namespace yarp {
    namespace dev {
        
        bool GazeboYarpControlBoardDriver::setPid (int j, const Pid &pid) { return false; }
        bool GazeboYarpControlBoardDriver::setPids (const Pid *pids) { return false; }
        bool GazeboYarpControlBoardDriver::setReference (int j, double ref) { return false; }
        bool GazeboYarpControlBoardDriver::setReferences (const double *refs) { return false; }
        bool GazeboYarpControlBoardDriver::setErrorLimit (int j, double limit) { return false; }
        bool GazeboYarpControlBoardDriver::setErrorLimits (const double *limits) { return false; }
        bool GazeboYarpControlBoardDriver::getError (int j, double *err) { return false; }
        bool GazeboYarpControlBoardDriver::getErrors (double *errs) { return false; }
        bool GazeboYarpControlBoardDriver::getPid (int j, Pid *pid) { return false; }
        bool GazeboYarpControlBoardDriver::getPids (Pid *pids) { return false; }
        bool GazeboYarpControlBoardDriver::getReference (int j, double *ref) { return false; }
        bool GazeboYarpControlBoardDriver::getReferences (double *refs) { return false; }
        bool GazeboYarpControlBoardDriver::getErrorLimit (int j, double *limit) { return false; }
        bool GazeboYarpControlBoardDriver::getErrorLimits (double *limits) { return false; }
        bool GazeboYarpControlBoardDriver::resetPid (int j) { return false; }
        bool GazeboYarpControlBoardDriver::disablePid (int j) { return false; }
        bool GazeboYarpControlBoardDriver::enablePid (int j) { return false; }
        bool GazeboYarpControlBoardDriver::setOffset (int j, double v) { return false; }
        
    }
}
