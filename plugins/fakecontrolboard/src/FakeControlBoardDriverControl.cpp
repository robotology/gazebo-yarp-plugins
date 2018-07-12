/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "FakeControlBoardDriver.h"

using namespace yarp::dev;

//////////////////////////////////////////////////////////////////////////////
// Helper functions
//////////////////////////////////////////////////////////////////////////////

bool GazeboYarpFakeControlBoardDriver::getZero(int j, double *ref)
{
    if (ref && j >= 0 && j < (int)m_numberOfJoints) {
        *ref = 0.0;
        return true;
    }
    return false;
}

bool GazeboYarpFakeControlBoardDriver::getZero(double * encs)
{
    if (!encs) return false;
    for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
        encs[i] = 0.0;
    }
    return true;
}

bool GazeboYarpFakeControlBoardDriver::getZero(const int n_joint, const int *joints, double *refs)
{
    if (!joints || !refs) return false;
    bool ret = true;
    for (int i = 0; i < n_joint && ret; i++)
    {
        ret = getZero(joints[i], refs+i);
    }
    return ret;
}

bool GazeboYarpFakeControlBoardDriver::getTrue(int j, bool *flag)
{
    if (flag && j >= 0 && j < (int)m_numberOfJoints) {
        *flag = true;
        return true;
    }
    return false;
}

bool GazeboYarpFakeControlBoardDriver::getTrue(bool * flags)
{
    if (!flags) return false;
    for (unsigned int i = 0; i < m_numberOfJoints; ++i) {
        flags[i] = true;
    }
    return true;
}

bool GazeboYarpFakeControlBoardDriver::getTrueIfArgumentIsZero(const int n_joints)
{
    return (n_joints == 0);
}

//////////////////////////////////////////////////////////////////////////////
// IPositionControl
//////////////////////////////////////////////////////////////////////////////

bool GazeboYarpFakeControlBoardDriver::getAxes(int *ax) // WORKS
{
    if (!ax) return false;
    *ax = m_numberOfJoints;
    return true;
}

// Method that always return true (to avoid errors in yarpmotorgui)
bool GazeboYarpFakeControlBoardDriver::checkMotionDone(int j, bool *flag) {return getTrue(j,flag);}
bool GazeboYarpFakeControlBoardDriver::checkMotionDone(bool *flag) {return getTrue(flag);}
bool GazeboYarpFakeControlBoardDriver::getTargetPosition(const int joint, double *ref) {return getZero(joint,ref);}
bool GazeboYarpFakeControlBoardDriver::getTargetPositions(double *refs) {return getZero(refs);}
bool GazeboYarpFakeControlBoardDriver::getTargetPositions(const int n_joint, const int *joints, double *refs) {return getZero(n_joint,joints,refs);}
bool GazeboYarpFakeControlBoardDriver::getRefSpeed(int j, double *ref) {return getZero(j,ref);}
bool GazeboYarpFakeControlBoardDriver::getRefSpeeds(double *spds) {return getZero(spds);}

// Setter methods with three arguments are critical :
// if they are called with a list of zero joints,
// they should return true, otherwise false
bool GazeboYarpFakeControlBoardDriver::positionMove(const int n_joint, const int *joints, const double *refs) {return getTrueIfArgumentIsZero(n_joint);}
bool GazeboYarpFakeControlBoardDriver::relativeMove(const int n_joint, const int *joints, const double *deltas) {return getTrueIfArgumentIsZero(n_joint);}
bool GazeboYarpFakeControlBoardDriver::checkMotionDone(const int n_joint, const int *joints, bool *flags) {return getTrueIfArgumentIsZero(n_joint);}
bool GazeboYarpFakeControlBoardDriver::setRefSpeeds(const int n_joint, const int *joints, const double *spds) {return getTrueIfArgumentIsZero(n_joint);}
bool GazeboYarpFakeControlBoardDriver::setRefAccelerations(const int n_joint, const int *joints, const double *accs) {return getTrueIfArgumentIsZero(n_joint);}
bool GazeboYarpFakeControlBoardDriver::getRefSpeeds(const int n_joint, const int *joints, double *spds) {return getTrueIfArgumentIsZero(n_joint);}
bool GazeboYarpFakeControlBoardDriver::getRefAccelerations(const int n_joint, const int *joints, double *accs) {return getTrueIfArgumentIsZero(n_joint);}
bool GazeboYarpFakeControlBoardDriver::stop(const int n_joint, const int *joints) {return getTrueIfArgumentIsZero(n_joint);}
bool GazeboYarpFakeControlBoardDriver::setPositions(const int n_joint, const int *joints, const double *refs) {return getTrueIfArgumentIsZero(n_joint);}

// Method that always return false
bool GazeboYarpFakeControlBoardDriver::setRefSpeed(int j, double sp) {return false;}
bool GazeboYarpFakeControlBoardDriver::positionMove(int j, double ref) {return false;}
bool GazeboYarpFakeControlBoardDriver::stop(int j) {return false;}
bool GazeboYarpFakeControlBoardDriver::stop() {return false;}
bool GazeboYarpFakeControlBoardDriver::positionMove(const double *refs) {return false;}
bool GazeboYarpFakeControlBoardDriver::relativeMove(int j, double delta) {return false;}
bool GazeboYarpFakeControlBoardDriver::relativeMove(const double *deltas) {return false;}
bool GazeboYarpFakeControlBoardDriver::setRefSpeeds(const double *spds) {return false;}
bool GazeboYarpFakeControlBoardDriver::setRefAcceleration(int j, double acc) {return false;}
bool GazeboYarpFakeControlBoardDriver::setRefAccelerations(const double *accs) {return false;}
bool GazeboYarpFakeControlBoardDriver::getRefAcceleration(int j, double *acc) {return false;}
bool GazeboYarpFakeControlBoardDriver::getRefAccelerations(double *accs) {return false;}
bool GazeboYarpFakeControlBoardDriver::setPosition(int j, double ref) {return false;}
bool GazeboYarpFakeControlBoardDriver::setPositions(const double *refs) {return false;}

//////////////////////////////////////////////////////////////////////////////
// IVelocityControl
//////////////////////////////////////////////////////////////////////////////

bool GazeboYarpFakeControlBoardDriver::getRefVelocity(const int joint, double *vel) { return getZero(joint,vel); }
bool GazeboYarpFakeControlBoardDriver::getRefVelocities(double *vels) {return getZero(vels);}
bool GazeboYarpFakeControlBoardDriver::getRefVelocities(const int n_joint, const int *joints, double *vels)
{
    return getZero(n_joint,joints,vels);
}

bool GazeboYarpFakeControlBoardDriver::velocityMove(const int n_joint, const int *joints, const double *spds) {return getTrueIfArgumentIsZero(n_joint);}

bool GazeboYarpFakeControlBoardDriver::velocityMove(int j, double sp) {return false;}
bool GazeboYarpFakeControlBoardDriver::velocityMove(const double *sp) {return false;}

//////////////////////////////////////////////////////////////////////////////
// ITorqueControl
//////////////////////////////////////////////////////////////////////////////

bool GazeboYarpFakeControlBoardDriver::getTorque(int j, double* t)
{
    if (t && j >= 0 && j < (int)m_numberOfJoints) {
        *t = m_torques[j];
        return true;
    }
    return false;
}

bool GazeboYarpFakeControlBoardDriver::getTorques(double* t)
{
    if (!t) return false;
    for (unsigned int j = 0; j < m_numberOfJoints; ++j) {
        t[j] = m_torques[j];
    }
    return true;
}

// Methods always returning true
bool GazeboYarpFakeControlBoardDriver::getRefTorque(int j, double* rt) {return  getZero(j,rt);}
bool GazeboYarpFakeControlBoardDriver::getRefTorques(double* rt) { return getZero(rt); }

// Methods always returning false
bool GazeboYarpFakeControlBoardDriver::setRefTorque(int, double) {return false;}
bool GazeboYarpFakeControlBoardDriver::setRefTorques(const double*) {return false;}
bool GazeboYarpFakeControlBoardDriver::getTorqueRange(int, double*, double *){return false;}
bool GazeboYarpFakeControlBoardDriver::getTorqueRanges(double *, double *){return false;}
