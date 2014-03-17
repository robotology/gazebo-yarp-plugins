/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include <gazebo_yarp_plugins/IMUDriver.h>
#include <gazebo_yarp_plugins/Handler.hh>

using namespace yarp::dev;

#define toDeg(X) (X*180.0/M_PI)

GazeboYarpIMUDriver::GazeboYarpIMUDriver()
{
}


GazeboYarpIMUDriver::~GazeboYarpIMUDriver()
{

}


/**
 *
 * Export an inertial sensor.
 * The network interface is a single Port.
 * We will stream bottles with 12 floats:
 * 0  1   2  = Euler orientation data (Kalman filter processed)
 * 3  4   5  = Calibrated 3-axis (X, Y, Z) acceleration data
 * 6  7   8  = Calibrated 3-axis (X, Y, Z) gyroscope data
 * 9 10 11   = Calibrated 3-axis (X, Y, Z) magnetometer data
 *
 * \todo check orientation data
 */
void GazeboYarpIMUDriver::onUpdate(const gazebo::common::UpdateInfo & /*_info*/)
{
    gazebo::math::Vector3 euler_orientation;
    gazebo::math::Vector3 linear_acceleration;
    gazebo::math::Vector3 angular_velocity;
        
    euler_orientation = this->parentSensor->GetOrientation().GetAsEuler();
    linear_acceleration = this->parentSensor->GetLinearAcceleration();
    angular_velocity = this->parentSensor->GetAngularVelocity();
    
    /** \todo ensure that the timestamp is the right one */
    last_timestamp.update(this->parentSensor->GetLastUpdateTime().Double());
    
        
    int i=0;
    
    data_mutex.wait();
    
    for(i = 0; i < 3; i++ ) {
        imu_data[0+i] = toDeg(euler_orientation[i]);
    }
    
    for(i = 0; i < 3; i++ ) {
        imu_data[3+i] = linear_acceleration[i];
    }
    
    for(i = 0; i < 3; i++ ) {
        imu_data[6+i] = toDeg(angular_velocity[i]);
    }
    
    data_mutex.post();
    
    return;
}
    
//DEVICE DRIVER
bool GazeboYarpIMUDriver::open(yarp::os::Searchable& config)
{
    data_mutex.wait();
    imu_data.resize(yarp_imu_nr_of_channels,0.0);
    data_mutex.post();
    
    //Get gazebo pointers
    std::string sensorScopedName (config.find(yarp_scopedname_parameter.c_str()).asString().c_str());
    std::cout << "GazeboYarpIMUDriver is looking for sensor " << sensorScopedName << "...\n";
    
    parentSensor = (gazebo::sensors::ImuSensor*)gazebo::GazeboYarpPluginHandler::getHandler()->getSensor(sensorScopedName);
    
    if(NULL == parentSensor)
    {
        std::cout << "GazeboYarpIMUDriver Error: IMU sensor was not found\n";
        return false;
    }
    
    //Connect the driver to the gazebo simulation
    this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboYarpIMUDriver::onUpdate, this, _1));
  
    return true;
}

bool GazeboYarpIMUDriver::close()
{
    if (this->updateConnection.get()) {
        gazebo::event::Events::DisconnectWorldUpdateBegin (this->updateConnection);
        this->updateConnection = ConnectionPtr();
    }
    return true;
}
    
//GENERIC SENSOR
bool GazeboYarpIMUDriver::read(yarp::sig::Vector &out)
{    
    if( (int)imu_data.size() != yarp_imu_nr_of_channels  ) {
        return false;
    }
    
    //< \todo TODO this should be avoided by properly modifyng the wrapper
    if( out.size() != imu_data.size() ) {
        out.resize(imu_data.size());
    }
    
    data_mutex.wait();
    out = imu_data;
    data_mutex.post();
    
    return true;
}

bool GazeboYarpIMUDriver::getChannels(int *nc)
{
    *nc = yarp_imu_nr_of_channels;
    return true;
}

bool GazeboYarpIMUDriver::calibrate(int ch, double v)
{
    return true; //Calibration is not needed in simulation
}

//PRECISELY TIMED
yarp::os::Stamp GazeboYarpIMUDriver::getLastInputStamp()
{
    return last_timestamp;
}
