/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include <gazebo_yarp_plugins/ForceTorqueDriver.h>
#include <gazebo_yarp_plugins/Handler.hh>

using namespace yarp::dev;


GazeboYarpForceTorqueDriver::GazeboYarpForceTorqueDriver()
{

}


GazeboYarpForceTorqueDriver::~GazeboYarpForceTorqueDriver()
{

}


/**
 *
 * Export a force/torque sensor.
 * 
 * \todo check forcetorque data
 */
void GazeboYarpForceTorqueDriver::onUpdate(const gazebo::common::UpdateInfo & /*_info*/)
{
    gazebo::math::Vector3 force;
    gazebo::math::Vector3 torque;
    
    force = this->parentSensor->GetForce();
    torque = this->parentSensor->GetTorque();
    
    /** \todo ensure that the timestamp is the right one */
    last_timestamp.update(this->parentSensor->GetLastUpdateTime().Double());
    
    int i=0;
    
    data_mutex.wait();
    
    for(i = 0; i < 3; i++ ) {
        forcetorque_data[0+i] = force[i];
    }
    
    for(i = 0; i < 3; i++ ) {
        forcetorque_data[3+i] = torque[i];
    }
    
    data_mutex.post();
    
    return;
}
    
//DEVICE DRIVER
bool GazeboYarpForceTorqueDriver::open(yarp::os::Searchable& config)
{
    std::cout << "GazeboYarpForceTorqueDriver::open() called" << std::endl;
  
    data_mutex.wait();
    forcetorque_data.resize(yarp_forcetorque_nr_of_channels,0.0);
    data_mutex.post();
    
    //Get gazebo pointers
    std::string sensorScopedName (config.find(yarp_scopedname_parameter.c_str()).asString().c_str());
    std::cout << "GazeboYarpForceTorqueDriver::open( is looking for sensor " << sensorScopedName << "...\n";
    
    parentSensor = (gazebo::sensors::ForceTorqueSensor*) gazebo::GazeboYarpPluginHandler::getHandler()->getSensor(sensorScopedName);
    
    if(NULL == parentSensor)
    {
        std::cout << "Error, ForceTorque sensor was not found\n";
        return AS_ERROR;
    }
    
    //Connect the driver to the gazebo simulation
    this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin (
                                 boost::bind ( &GazeboYarpForceTorqueDriver::onUpdate, this, _1 ) );
  
    std::cout << "GazeboYarpForceTorqueDriver::open() returning true" << std::endl;
    return true;
}

bool GazeboYarpForceTorqueDriver::close()
{
    gazebo::event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
    return true;
}
    
//ANALOG SENSOR
int GazeboYarpForceTorqueDriver::read(yarp::sig::Vector &out)
{
    ///< \todo TODO in my opinion the reader should care of passing a vector of the proper dimension to the driver, but apparently this is not the case
    /*
    if( (int)forcetorque_data.size() != yarp_forcetorque_nr_of_channels ||
        (int)out.size() != yarp_forcetorque_nr_of_channels ) {
        return AS_ERROR;
    }
    */
    
   if( (int)forcetorque_data.size() != yarp_forcetorque_nr_of_channels ) {
        return AS_ERROR;
   }
   
   if( (int)out.size() != yarp_forcetorque_nr_of_channels ) {
       out.resize(yarp_forcetorque_nr_of_channels);
   }
    
    
    data_mutex.wait();
    out = forcetorque_data;
    data_mutex.post();
    
    return AS_OK;
}

int GazeboYarpForceTorqueDriver::getChannels()
{
    return yarp_forcetorque_nr_of_channels;
}

int GazeboYarpForceTorqueDriver::getState(int ch)
{
    printf("getstate\n");
    return AS_OK;
}

int GazeboYarpForceTorqueDriver::calibrateSensor()
{
    return AS_OK;
}

int GazeboYarpForceTorqueDriver::calibrateSensor(const yarp::sig::Vector& value)
{
    return AS_OK;
}

int GazeboYarpForceTorqueDriver::calibrateChannel(int ch)
{
    return AS_OK;
}

int GazeboYarpForceTorqueDriver::calibrateChannel(int ch, double v)
{
    return AS_OK;
}

//PRECISELY TIMED
yarp::os::Stamp GazeboYarpForceTorqueDriver::getLastInputStamp()
{
    return last_timestamp;
}