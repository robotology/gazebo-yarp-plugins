/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "LaserSensorDriver.h"
#include <GazeboYarpPlugins/Handler.hh>


#include <boost/bind/bind.hpp>
#include <gazebo/sensors/sensors.hh>
#include <yarp/os/LogStream.h>

using namespace boost::placeholders;

using namespace yarp::dev;

const std::string YarpLaserSensorScopedName = "sensorScopedName";
namespace {
    YARP_LOG_COMPONENT(GAZEBOLASER, "gazebo-yarp-plugins.plugins.GazeboYarpLaserSensor")
}

GazeboYarpLaserSensorDriver::GazeboYarpLaserSensorDriver() {}
GazeboYarpLaserSensorDriver::~GazeboYarpLaserSensorDriver() {}

/**
 *
 * Export a laser sensor.
 *
 */
void GazeboYarpLaserSensorDriver::onUpdate(const gazebo::common::UpdateInfo& _info)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    std::vector<double> tmp;
    this->m_parentSensor->Ranges(tmp);
    
    if (tmp.size() != m_laser_data.size())
    {
        yCError(GAZEBOLASER) << "size error";
    }

#if 0
    //looking for an efficient way to convert a std::vector into a yarp::sig::vector
    double* ptr = m_laser_data.data();
    ptr = tmp.data();
#else
    for (size_t i=0; i< tmp.size(); i++)
    {
        m_laser_data[i]=tmp[i];
    }
#endif
    
    this->applyLimitsOnLaserData();
    m_lastTimestamp.update(_info.simTime.Double());
    m_first_run = false;
    return;
}

//DEVICE DRIVER
bool GazeboYarpLaserSensorDriver::open(yarp::os::Searchable& config)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    //Get gazebo pointers
    std::string sensorScopedName(config.find(YarpLaserSensorScopedName.c_str()).asString().c_str());

    m_parentSensor = dynamic_cast<gazebo::sensors::RaySensor*>(GazeboYarpPlugins::Handler::getHandler()->getSensor(sensorScopedName));
    m_first_run = true;
    
    if (!m_parentSensor)
    {
        yCError(GAZEBOLASER) << "Error, sensor" <<  sensorScopedName << "was not found" ;
        return  false ;
    }

    //Connect the driver to the gazebo simulation
    this->m_updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboYarpLaserSensorDriver::onUpdate, this, _1));

    //this block of parameters is obtained from gazebo
    m_gazebo_max_angle = m_parentSensor->AngleMax().Degree();            //m_max_angles is expressed in degrees
    m_gazebo_min_angle = m_parentSensor->AngleMin().Degree();            //m_min_angles is expressed in degrees
    m_gazebo_max_range = m_parentSensor->RangeMax();                     //m
    m_gazebo_min_range = m_parentSensor->RangeMin();                     //m
    m_gazebo_resolution = m_parentSensor->AngleResolution()*180.0/M_PI;  //m_resolution is expressed in degrees
    m_gazebo_samples   = m_parentSensor->RangeCount();
    m_gazebo_scan_rate = m_parentSensor->UpdateRate();

     //parse all the parameters related to the linear/angular range of the sensor
    if (this->parseConfiguration(config) == false)
    {
        yCError(GAZEBOLASER) << "error parsing parameters";
        return false;
    }

    return true;
}

bool GazeboYarpLaserSensorDriver::close()
{
    this->m_updateConnection.reset();

    return true;
}

bool GazeboYarpLaserSensorDriver::acquireDataFromHW()
{
    //Not used.
    //GazeboYarpLaserSensorDriver::onUpdate manages the all the logic
    return true;
}

//PRECISELY TIMED
yarp::os::Stamp GazeboYarpLaserSensorDriver::getLastInputStamp()
{
    return m_lastTimestamp;
}

bool GazeboYarpLaserSensorDriver::setDistanceRange (double min, double max)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    yCError(GAZEBOLASER) << "setDistanceRange() Not yet implemented";
    return false;
}

bool GazeboYarpLaserSensorDriver::setScanLimits (double min, double max)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    yCError(GAZEBOLASER) << "setScanLimits() Not yet implemented";
    return false;
}

bool GazeboYarpLaserSensorDriver::setHorizontalResolution (double step)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    yCError(GAZEBOLASER) << "setHorizontalResolution() Not yet implemented";
    return false;
}

bool GazeboYarpLaserSensorDriver::setScanRate (double rate)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    if (rate<0)
    {
      yCError(GAZEBOLASER) << "Invalid setScanRate";
      return false;
    }
    m_parentSensor->SetUpdateRate(rate);
    m_scan_rate = rate;
    return true;
}
