/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include "gazebo_yarp_plugins/ControlBoardDriver.h"
#include "gazebo_yarp_plugins/Handler.hh"


using namespace yarp::dev;
using namespace gazebo;


bool GazeboYarpControlBoardDriver::open(yarp::os::Searchable& config) 
{
    plugin_parameters.fromString(config.toString().c_str());

    std::string robotName (plugin_parameters.find("robotScopedName").asString().c_str());
    std::cout << "DeviceDriver is looking for robot " << robotName << "...\n";

    _robot = GazeboYarpPlugins::Handler::getHandler()->getRobot(robotName);
    if(NULL == _robot)
    {
        std::cout << "GazeboYarpControlBoardDriver error: robot was not found\n";
        return false;
    }
    
    return gazebo_init();
}

bool GazeboYarpControlBoardDriver::close()
{
    //unbinding events
    gazebo::event::Events::DisconnectWorldUpdateBegin (this->updateConnection);
    
    delete [] control_mode;
    delete [] motion_done;
    return true;
}
