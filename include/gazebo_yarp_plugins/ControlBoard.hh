/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef GAZEBOYARP_CONTROLBOARD_HH
#define GAZEBOYARP_CONTROLBOARD_HH

#include <gazebo/gazebo.hh>
#include <string>

#include <yarp/os/Network.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/dev/PolyDriverList.h>

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

    void Init();

    virtual ~GazeboYarpControlBoard();

    /**
     * Saves the gazebo pointer, creates the device driver
     */
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    
private:
    
    yarp::os::Network _yarp;
    yarp::dev::PolyDriver _wrapper;
    yarp::dev::IMultipleWrapper *_iWrap;
    yarp::dev::PolyDriver _controlBoard;
    yarp::os::Property _parameters;
    
    std::string _robotName;

};

}

#endif
