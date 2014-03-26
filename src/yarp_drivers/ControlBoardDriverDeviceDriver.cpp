/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include <gazebo_yarp_plugins/ControlBoardDriver.h>

#include "gazebo_yarp_plugins/Handler.hh"


using namespace yarp::dev;
using namespace gazebo;


bool GazeboYarpControlBoardDriver::open(yarp::os::Searchable& config) 
{
    plugin_parameters.fromString(config.toString().c_str());

    std::string robotName (plugin_parameters.find("robotScopedName").asString().c_str());
    std::cout << "DeviceDriver is looking for robot " << robotName << "...\n";

    _robot = GazeboYarpPluginHandler::getHandler()->getRobot(robotName);
    if(NULL == _robot)
    {
        std::cout << "Error, robot was not found\n";
        return false;
    }
    
    return gazebo_init() && RateThread::start();
}

bool GazeboYarpControlBoardDriver::close()
{
    this->askToStop(); //stop thread.
    //unbinding events
    gazebo::event::Events::DisconnectWorldUpdateBegin (this->updateConnection);
    
    delete [] control_mode;
    delete [] motion_done;
    return true;
}


//We need a thread to publish some extra information like joint torques and velocities.
bool GazeboYarpControlBoardDriver::threadInit()
{
    //Find robot prefix used in wrapper
    yarp::os::Value &wrapper_port_name = plugin_parameters.findGroup("WRAPPER").find("name");
    if(wrapper_port_name.isNull())
        printf("\n\nerror WRAPPER name not found\n %s\n\n", plugin_parameters.toString().c_str());

    return true;
}

void GazeboYarpControlBoardDriver::afterStart(bool s)
{

}

void GazeboYarpControlBoardDriver::run()
{

}

void GazeboYarpControlBoardDriver::threadRelease()
{

}
