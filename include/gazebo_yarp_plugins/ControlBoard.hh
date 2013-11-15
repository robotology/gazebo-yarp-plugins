/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#pragma GCC diagnostic pop

#include <yarp/os/Network.h>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <gazebo_yarp_plugins/ControlBoardDriver.h>


#define ms(X) (X * 1000.0)
#define toRad(X) (X*M_PI/180.0)

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

    ~GazeboYarpControlBoard();

    /**
     * Saves the gazebo pointer, creates the device driver
     */
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    
private:
    /**
      * Simulation Time Step in ms
      */
    //double _dT;
    physics::ModelPtr _robot;
    yarp::os::Network _yarp;
    yarp::dev::PolyDriver _driver;
    yarp::os::Property _parameters;
   // event::ConnectionPtr updateConnection; // Pointer to the update event connection
};

}
