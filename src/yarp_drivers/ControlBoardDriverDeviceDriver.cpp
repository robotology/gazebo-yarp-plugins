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
    
    gazebo_init();
    return RateThread::start();
}



bool GazeboYarpControlBoardDriver::close() //NOT IMPLEMENTED
{
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
    
    std::stringstream port_name_torque;
    std::stringstream port_name_torque_rpc;

    port_name_torque<<"/"<<wrapper_port_name.toString().c_str()<<"/analog:o/torques";
    port_name_torque_rpc << port_name_torque.str().c_str() << "/rpc:i";
    _joint_torq_port.open(port_name_torque.str().c_str());
    _joint_torq_port_rpc.open(port_name_torque_rpc.str().c_str());

    std::stringstream port_name_speed;
    std::stringstream port_name_speed_rpc;
    port_name_speed<<"/"<<wrapper_port_name.toString().c_str()<<"/analog:o/speed";
    port_name_speed_rpc << port_name_speed.str().c_str() << "/rpc:i";
    _joint_speed_port.open(port_name_speed.str().c_str());
    _joint_speed_port_rpc.open(port_name_speed_rpc.str().c_str());

    return true;
}

void GazeboYarpControlBoardDriver::afterStart(bool s)
{
    if(s)
        printf("TorqueAndSpeedPublisher started successfully\n");
    else
        printf("TorqueAndSpeedPublisher did not start\n");
}

void GazeboYarpControlBoardDriver::run()
{
    yarp::os::Bottle bot1;
    yarp::os::Bottle bot2;
    bot1.clear();
    bot2.clear();
    for(unsigned int j = 0; j < _controlboard_number_of_joints; ++j)
    {
        bot1.addDouble(torque[j]);
        bot2.addDouble(speed[j]);
    }
    _joint_torq_port.write(bot1);
    _joint_speed_port.write(bot2);
}

void GazeboYarpControlBoardDriver::threadRelease()
{
    printf("Goodbye from TorqueAndSpeedPublisher\n");
}
