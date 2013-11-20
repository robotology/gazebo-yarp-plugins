/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include <gazebo_yarp_plugins/ControlBoardDriver.h>

#include <boost/archive/text_iarchive.hpp>
#include "gazebo_yarp_plugins/Handler.hh"


using namespace yarp::dev;
using namespace gazebo;


bool GazeboYarpControlBoardDriver::open(yarp::os::Searchable& config) 
{
    plugin_parameters.fromString(config.toString().c_str());

    //I love everything and every interface
    uintptr_t temp;
    std::istringstream iss(config.find("loving_gazebo_pointer").asString().c_str());
    boost::archive::text_iarchive archive(iss);
    try{
        archive>>temp;
    }
    catch (const char *ex)
    {
        //TODO
    }

    std::string robotName = plugin_parameters.find("robot").asString();
    std::cout << "DeviceDriver is looking for robot " << robotName << "...\n";
        gazebo::physics::ModelPtr tmp;

    _robot = GazeboYarpPluginHandler::getHandler()->getRobot(robotName);
    if(NULL == _robot)
    {
        std::cout << "Error, robot was not found\n";
        return false;
    }
    //_robot=reinterpret_cast<gazebo::physics::Model*>(temp);
    
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
    yarp::os::Value &name = plugin_parameters.find("name");
    
    if(name.isNull())
    {
        printf("\n\nerror name not found\n %s\n\n", plugin_parameters.toString().c_str());
    }
    
    std::stringstream port_name_torque;
    port_name_torque<<"/coman/"<< name.toString().c_str()<<"/analog/torques:o";
    _joint_torq_port.open(port_name_torque.str().c_str());
    std::stringstream port_name_speed;
    port_name_speed<<"/coman/"<< name.toString().c_str()<<"/analog/speeds:o";
    _joint_speed_port.open(port_name_speed.str().c_str());
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
    for(unsigned int j = 0; j < _robot_number_of_joints; ++j)
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
