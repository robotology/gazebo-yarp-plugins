/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_FAKECONTROLBOARD_HH
#define GAZEBOYARP_FAKECONTROLBOARD_HH

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
/// \class GazeboYarpFakeControlBoard
/// Gazebo Plugin emulating a fake yarp controlboard in Gazebo.
///
/// This device will expose a set of axis that are not actually present 
/// in the Gazebo simulation, and their control mode will always be idle 
/// their measured position will always be a given constant value, and 
/// their velocity, acceleration and torque will be 0.
///
/// It can be configurated using the yarpConfigurationFile sdf tag,
/// that contains a Gazebo URI pointing at a yarp .ini configuration file
/// containt the configuration parameters of the fake controlboard.
///
///
class GazeboYarpFakeControlBoard : public ModelPlugin
{
public:
    GazeboYarpFakeControlBoard();
    virtual ~GazeboYarpFakeControlBoard();

    /**
     * Saves the gazebo pointer, creates the device driver
     */
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

private:
    yarp::dev::PolyDriver m_wrapper;
    yarp::dev::IMultipleWrapper* m_iWrap;
    yarp::dev::PolyDriverList m_controlBoards;

    yarp::os::Property m_pluginParameters;

    std::string m_robotName;
};

}

#endif
