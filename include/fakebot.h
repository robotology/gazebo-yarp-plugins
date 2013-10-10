/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia iCub Facility & ADVR
 * Authors: Lorenzo Natale and Paul Fitzpatrick and Enrico Mingo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef FAKEBOT_H
#define FAKEBOT_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/Network.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/FrameGrabberInterfaces.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_world.h>

namespace yarp {
    namespace dev {
      class fakebot;
    }
}

class yarp::dev::fakebot : public DeviceDriver,
            public IPositionControl,
            public IVelocityControl,
            public IAmplifierControl,
            public IEncoders,
            public IControlCalibration2,
            public IControlLimits,
            public DeviceResponder,
            public IControlMode,
            private yarp::os::RateThread
{
public:
    fakebot():RateThread(0)
    {
        _robot = gazebo_world::getModel();

        _robot_number_of_joints = _robot->GetJoints().size();

        pos.size(_robot_number_of_joints);
        vel.size(_robot_number_of_joints);
        speed.size(_robot_number_of_joints);
        acc.size(_robot_number_of_joints);
        amp.size(_robot_number_of_joints);
        ref_speed.size(_robot_number_of_joints);
        ref_pos.size(_robot_number_of_joints);
        ref_acc.size(_robot_number_of_joints);
        max_pos.size(_robot_number_of_joints);
        min_pos.size(_robot_number_of_joints);
        joint_names.reserve(_robot_number_of_joints);

        setJointNames();
        setMinMaxPos();

        pos = 0;
        vel = 0;
        speed = 0;
        ref_speed=0;
        ref_pos=0;
        ref_acc=0;
        acc = 0;
        amp = 1; // initially on - ok for simulator

        control_mode=new int[_robot_number_of_joints];
        motion_done=new bool[_robot_number_of_joints];

        for(int j=0; j<_robot_number_of_joints; ++j)
            control_mode[j]=VOCAB_CM_POSITION;
    }

    ~fakebot()
    {
        delete [] control_mode;
        delete [] motion_done;
    }

    // thread
    virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();

    virtual bool open(yarp::os::Searchable& config);

    // IPositionControl etc.

    virtual bool getAxes(int *ax) {
        *ax = _robot_number_of_joints;
        return true;
    }

    virtual bool setPositionMode() {
        return true;
    }

    virtual bool positionMove(int j, double ref) {
        if (j<_robot_number_of_joints) {
            //ref_pos[j] = ref;
            gazebo::physics::JointControllerPtr p =_robot->GetJointController();
            gazebo::physics::JointPtr jj = _robot->GetJoint(joint_names[j]);
            p->SetJointPosition(jj, ref);
            p->Update();
        }
        return true;
    }

    virtual bool positionMove(const double *refs) {
        gazebo::physics::JointControllerPtr p =_robot->GetJointController();
        std::map<std::string, double> joints_map;
        for (int i=0; i<_robot_number_of_joints; ++i) {
            //ref_pos[i] = refs[i];
            joints_map[joint_names[i]] = refs[i];
        }
        p->SetJointPositions(joints_map);
        p->Update();
        return true;
    }

    virtual bool relativeMove(int j, double delta) {
        if (j<_robot_number_of_joints) {
            ref_pos[j] += delta;
        }
        return true;
    }


    virtual bool relativeMove(const double *deltas) {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            ref_pos[i] += deltas[i];
        }
        return true;
    }

    virtual bool checkMotionDone(int j, bool *flag) {
        *flag=motion_done[j];
        return true;
    }


    virtual bool checkMotionDone(bool *flag) {
        *flag=true;
        for(int j=0; j<_robot_number_of_joints; ++j)
         { *flag&&motion_done[j]; }

        return true;
    }


    virtual bool setRefSpeed(int j, double sp) {
        if (j<_robot_number_of_joints) {
            ref_speed[j] = sp;
        }
        return true;
    }

    virtual bool setRefSpeeds(const double *spds) {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            ref_speed[i] = spds[i];
        }
        return true;
    }


    virtual bool setRefAcceleration(int j, double acc) {
        if (j<_robot_number_of_joints) {
            ref_acc[j] = acc;
        }
        return true;
    }


    virtual bool setRefAccelerations(const double *accs) {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            ref_acc[i] = accs[i];
        }
        return true;
    }

    virtual bool getRefSpeed(int j, double *ref) {
        if (j<_robot_number_of_joints) {
            (*ref) = ref_speed[j];
        }
        return true;
    }


    virtual bool getRefSpeeds(double *spds) {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            spds[i] = ref_speed[i];
        }
        return true;
    }


    virtual bool getRefAcceleration(int j, double *acc) {
        if (j<_robot_number_of_joints) {
            (*acc) = ref_acc[j];
        }
        return true;
    }

    virtual bool getRefAccelerations(double *accs) {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            accs[i] = ref_acc[i];
        }
        return true;
    }


    virtual bool stop(int j) {
        ref_pos[j]=pos[j];
        return true;
    }


    virtual bool stop()
    {
        ref_pos=pos;
        return true;
    }


    virtual bool close() {
        return true;
    }

    virtual bool resetEncoder(int j) {
        if (j<_robot_number_of_joints) {
            pos[j] = 0;
         }
        return true;
    }

    virtual bool resetEncoders() {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            pos[i] = 0;
        }
        return true;
    }

    virtual bool setEncoder(int j, double val) {
        if (j<_robot_number_of_joints) {
            pos[j] = val;
        }
        return true;
    }

    virtual bool setEncoders(const double *vals) {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            pos[i] = vals[i];
        }
        return true;
    }

    virtual bool getEncoder(int j, double *v) {
        if (j<_robot_number_of_joints) {
            //(*v) = pos[j];
            gazebo::math::Angle a = _robot->GetJoint(joint_names[j])->GetAngle(0);
            (*v) = a.Radian();
        }

        return true;
    }

    virtual bool getEncoders(double *encs) {
        gazebo::math::Angle a;
        for (int i=0; i<_robot_number_of_joints; ++i) {
            //encs[i] = pos[i];
            a = _robot->GetJoint(joint_names[i])->GetAngle(0);
            encs[i] = a.Radian();
        }
        return true;
    }

    virtual bool getEncoderSpeed(int j, double *sp) {
        if (j<_robot_number_of_joints) {
            (*sp) = 0;
        }
        return true;
    }

    virtual bool getEncoderSpeeds(double *spds) {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            spds[i] = 0;
        }
        return true;
    }

    virtual bool getEncoderAcceleration(int j, double *spds) {
        if (j<_robot_number_of_joints) {
            (*spds) = 0;
        }
        return true;
    }

    virtual bool getEncoderAccelerations(double *accs) {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            accs[i] = 0;
        }
        return true;
    }

    virtual bool velocityMove(int j, double sp) {
        if (j<_robot_number_of_joints) {
            vel[j] = sp;
        }
        return true;
    }

    virtual bool velocityMove(const double *sp) {
        for (int i=0; i<_robot_number_of_joints; ++i) {
            vel[i] = sp[i];
        }
        return true;
    }

    virtual bool enableAmp(int j) {
        if (j<_robot_number_of_joints) {
            amp[j] = 1;
            control_mode[j]=VOCAB_CM_POSITION;
        }
        return true;
    }

    virtual bool disableAmp(int j) {
        if (j<_robot_number_of_joints) {
            amp[j] = 0;
            control_mode[j]=VOCAB_CM_IDLE;
        }
        return true;
    }

    virtual bool getCurrent(int j, double *val) {
        if (j<_robot_number_of_joints) {
            val[j] = amp[j];
        }
        return true;
    }

    virtual bool getCurrents(double *vals) {
        for (int i=0; i<_robot_number_of_joints; i++) {
            vals[i] = amp[i];
        }
        return true;
    }

    virtual bool setMaxCurrent(int j, double v) {
        return true;
    }

    virtual bool getAmpStatus(int *st) {
        *st = 0;
        return true;
    }

    virtual bool getAmpStatus(int k, int *v)
    {
        *v=0;
        return true;
    }

    virtual bool calibrate2(int j, unsigned int iv, double v1, double v2, double v3)
    {
        fprintf(stderr, "fakebot: calibrating joint %d with parameters %u %lf %lf %lf\n", j, iv, v1, v2, v3);
        return true;
    }

    virtual bool done(int j)
    {
        fprintf(stderr , "fakebot: calibration done on joint %d.\n", j);
        return true;
    }

    // IControlLimits
    virtual bool getLimits(int axis, double *min, double *max)
    {
        *min=min_pos[axis];
        *max=max_pos[axis];
        return true;
    }

    virtual bool setLimits(int axis, double min, double max)
    {
        max_pos[axis]=max;
        min_pos[axis]=min;
        return true;
    }


    // IControlMode
    virtual bool setPositionMode(int j){control_mode[j]=VOCAB_CM_POSITION;}
    virtual bool setVelocityMode(int j){control_mode[j]=VOCAB_CM_VELOCITY;}

    virtual bool setVelocityMode()
    {
       for(int j=0; j<_robot_number_of_joints; j++)
           { control_mode[j]=VOCAB_CM_VELOCITY;}
    }

    virtual bool setTorqueMode(int j){ return false; }
    virtual bool setImpedancePositionMode(int j){return false;}
    virtual bool setImpedanceVelocityMode(int j){return false;}
    virtual bool setOpenLoopMode(int j){return false; }
    virtual bool getControlMode(int j, int *mode){mode[j]=control_mode[j];}
    virtual bool getControlModes(int *modes)
    {
        for(int j=0;j<_robot_number_of_joints; ++j)
            {modes[j]=control_mode[j];}
        return true;
    }

private:
    unsigned int robot_refresh_period; //ms
    gazebo::physics::ModelPtr _robot;
    unsigned int _robot_number_of_joints;

    yarp::sig::Vector pos, vel, speed, acc, amp;
    yarp::sig::Vector ref_speed, ref_pos, ref_acc;
    yarp::sig::Vector max_pos, min_pos;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> back, fore;

    std::vector<std::string> joint_names;

    bool *motion_done;
    int  *control_mode;

    void setMinMaxPos()
    {
        std::cout<<"Joint Limits"<<std::endl;
        gazebo::physics::Joint_V joints = _robot->GetJoints();
        for(unsigned int i = 0; i < _robot_number_of_joints; ++i)
        {
            gazebo::physics::JointPtr j = joints[i];
            max_pos[i] = j->GetUpperLimit(0).Degree();
            min_pos[i] = j->GetLowerLimit(0).Degree();
            std::cout<<joint_names[i]<<" max_pos: "<<max_pos[i]<<" min_pos: "<<min_pos[i]<<std::endl;
        }
    }

    void setJointNames()
    {
        gazebo::physics::Joint_V joints = _robot->GetJoints();
        for(unsigned int i = 0; i < _robot_number_of_joints; ++i)
        {
            gazebo::physics::JointPtr j = joints[i];
            joint_names.push_back(j->GetName());
        }
    }
};

#endif
