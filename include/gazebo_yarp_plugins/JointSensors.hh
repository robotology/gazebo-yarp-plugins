/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef GAZEBOYARP_JOINTSENSORS_HH
#define GAZEBOYARP_JOINTSENSORS_HH

#include <gazebo/common/Plugin.hh>

#include <string>

#include <yarp/os/Network.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/dev/PolyDriverList.h>

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

    yarp::os::Network _yarp;
    yarp::dev::PolyDriver _jointsensors_wrapper;
    yarp::dev::IMultipleWrapper *_iWrap;
    yarp::dev::PolyDriver _jointsensors_driver;
    yarp::os::Property _parameters;

    std::string _robotName;

    bool plugin_loaded;
};

}

#endif
