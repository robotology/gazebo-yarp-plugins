#include "coman.h"

using namespace yarp::dev;

// IControlLimits
bool coman::getLimits(int axis, double *min, double *max) //NOT TESTED
{
    *min=min_pos[axis];
    *max=max_pos[axis];
    return true;
}

bool coman::setLimits(int axis, double min, double max) //NOT TESTED
{
    max_pos[axis]=max;
    min_pos[axis]=min;
    return true;
}

//Amplifiers
bool coman::enableAmp(int j) //NOT IMPLEMENTED
{
    if (j<_robot_number_of_joints) {
        amp[j] = 1;
        control_mode[j]=VOCAB_CM_POSITION;
    }
    return true;
}

bool coman::disableAmp(int j) //NOT IMPLEMENTED
{
    if (j<_robot_number_of_joints) {
        amp[j] = 0;
        control_mode[j]=VOCAB_CM_IDLE;
    }
    return true;
}

bool coman::getCurrent(int j, double *val) //NOT IMPLEMENTED
{
    if (j<_robot_number_of_joints) {
        val[j] = amp[j];
    }
    return true;
}

bool coman::getCurrents(double *vals) //NOT IMPLEMENTED
{
    for (int i=0; i<_robot_number_of_joints; i++) {
        vals[i] = amp[i];
    }
    return true;
}

bool coman::setMaxCurrent(int j, double v) //NOT IMPLEMENTED
{
    return true;
}

bool coman::getAmpStatus(int *st) //NOT IMPLEMENTED
{
    *st = 0;
    return true;
}

bool coman::getAmpStatus(int k, int *v) //NOT IMPLEMENTED
{
    *v=0;
    return true;
}

bool coman::calibrate2(int j, unsigned int iv, double v1, double v2, double v3) //NOT IMPLEMENTED
{
    fprintf(stderr, "fakebot: calibrating joint %d with parameters %u %lf %lf %lf\n", j, iv, v1, v2, v3);
    return true;
}

bool coman::done(int j) // NOT IMPLEMENTED
{
    fprintf(stderr , "fakebot: calibration done on joint %d.\n", j);
    return true;
}

