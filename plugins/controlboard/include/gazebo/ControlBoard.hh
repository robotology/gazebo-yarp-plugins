/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_CONTROLBOARD_HH
#define GAZEBOYARP_CONTROLBOARD_HH

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
/// \class GazeboYarpControlBoard
/// Gazebo Plugin emulating the yarp controlBoard device in Gazebo.
///.
/// It can be configurated using the yarpConfigurationFile sdf tag,
/// that contains a Gazebo URI pointing at a yarp .ini configuration file
/// containt the configuration parameters of the controlBoard
///
/// The gazebo plugin is the "main" of the yarp device,
/// so what it should is to initialize the device, copy the
/// gazebo pointer, and return
///
/// The device will receive the gazebo pointer, parse the model,
/// and wait for yarp connections and the gazebo wait event.
///
class GazeboYarpControlBoard : public ModelPlugin
{
public:
    GazeboYarpControlBoard();
    virtual ~GazeboYarpControlBoard();

    /**
     * Saves the gazebo pointer, creates the device driver
     */
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

private:
    yarp::dev::PolyDriver m_wrapper;
    yarp::dev::IMultipleWrapper* m_iWrap;
    yarp::dev::PolyDriverList m_controlBoards;

    yarp::os::Property m_parameters;

    std::string m_robotName;
};

}

#endif
