/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_JOINTSENSORS_HH
#define GAZEBOYARP_JOINTSENSORS_HH

#include <gazebo/common/Plugin.hh>

#include <string>

#include <yarp/dev/PolyDriverList.h>

namespace yarp {
    namespace dev {
        class IMultipleWrapper;
    }
}

namespace gazebo
{
/// \class GazeboYarpJointSensors
/// Gazebo plugin that creates a yarp streaming sensors (IAnalogSensor)
/// for streaming joint sensor measures (position, speed or torque)
///
class GazeboYarpJointSensors : public ModelPlugin
{
public:
    GazeboYarpJointSensors();

    virtual ~GazeboYarpJointSensors();

    /**
     * Saves the gazebo pointer, creates the device driver
     */
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

private:
    yarp::dev::PolyDriver m_jointsensorsWrapper;
    yarp::dev::IMultipleWrapper* m_iWrap;
    yarp::dev::PolyDriver m_jointsensorsDriver;

    std::string m_robotName;
};

}

#endif
