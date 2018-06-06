/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "LaserSensorDriver.h"
#include <GazeboYarpPlugins/Handler.hh>

#include <yarp/os/LogStream.h>
#include <yarp/os/LockGuard.h>
#include <gazebo/sensors/sensors.hh>
#include <yarp/os/LogStream.h>

using namespace yarp::dev;

const std::string YarpLaserSensorScopedName = "sensorScopedName";

GazeboYarpLaserSensorDriver::GazeboYarpLaserSensorDriver() {}
GazeboYarpLaserSensorDriver::~GazeboYarpLaserSensorDriver() {}

/**
 *
 * Export a force/torque sensor.
 *
 * \todo check forcetorque data
 */
void GazeboYarpLaserSensorDriver::onUpdate(const gazebo::common::UpdateInfo& _info)
{
    yarp::os::LockGuard guard(m_mutex);

    this->m_parentSensor->Ranges(m_sensorData);


    m_lastTimestamp.update(_info.simTime.Double());
    m_first_run = false;
    return;
}

//DEVICE DRIVER
bool GazeboYarpLaserSensorDriver::open(yarp::os::Searchable& config)
{
    yarp::os::LockGuard guard(m_mutex);

    //Get gazebo pointers
    std::string sensorScopedName(config.find(YarpLaserSensorScopedName.c_str()).asString().c_str());

    m_parentSensor = dynamic_cast<gazebo::sensors::RaySensor*>(GazeboYarpPlugins::Handler::getHandler()->getSensor(sensorScopedName));
    m_first_run = true;
    
    if (!m_parentSensor)
    {
        yError() << "Error, sensor" <<  sensorScopedName << "was not found" ;
        return  false ;
    }

    //Connect the driver to the gazebo simulation
    this->m_updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboYarpLaserSensorDriver::onUpdate, this, _1));


    m_max_angle = m_parentSensor->AngleMax().Degree();            //m_max_angles is expressed in degrees
    m_min_angle = m_parentSensor->AngleMin().Degree();            //m_min_angles is expressed in degrees
    m_max_gazebo_range = m_parentSensor->RangeMax();              //m
    m_min_gazebo_range = m_parentSensor->RangeMin();              //m
    m_resolution = m_parentSensor->AngleResolution()*180.0/M_PI;  //m_resolution is expressed in degrees
    m_samples   = m_parentSensor->RangeCount();
    m_rate      = m_parentSensor->UpdateRate();

    m_sensorData.resize(m_samples, 0.0);

    bool bg = config.check("GENERAL");
    if (bg != false)
    {
        yarp::os::Searchable& general_config = config.findGroup("GENERAL");
        if (general_config.check("clip_min_range")==false) {yError() << "Missing clip_min_range"; return false; }
        m_min_clip_range = general_config.find("clip_min_range").asDouble();
        if (general_config.check("clip_max_range")==false) {yError() << "Missing clip_max_range"; return false; }
        m_max_clip_range = general_config.find("clip_max_range").asDouble();
        if (general_config.check("discard_min_range")==false) {yError() << "Missing discard_min_range"; return false; }
        m_min_discard_range = general_config.find("discard_min_range").asDouble();
        if (general_config.check("discard_max_range")==false) {yError() << "Missing discard_max_range"; return false; }
        m_max_discard_range = general_config.find("discard_max_range").asDouble();
        if (general_config.check("enable_clip_range")==false) {yError() << "Missing enable_clip_range"; return false; }
        m_enable_clip_range = (general_config.find("enable_clip_range").asInt32()==1);
        if (general_config.check("enable_discard_range")==false) {yError() << "Missing enable_discard_range"; return false; }
        m_enable_discard_range = (general_config.find("enable_discard_range").asInt32()==1);

        if (m_enable_clip_range==true && m_enable_discard_range)
        {
            yError() << "enable_clip_range and enable_discard_range both enabled! Choose one";
            return false;
        }
    }
    else
    {
        yError() << "Missing GENERAL section";
        return false;
    }

    bool bs = config.check("SKIP");
    if (bs != false)
    {
        yarp::os::Searchable& skip_config = config.findGroup("SKIP");
        yarp::os::Bottle mins = skip_config.findGroup("min");
        yarp::os::Bottle maxs = skip_config.findGroup("max");
        size_t s_mins = mins.size();
        size_t s_maxs = mins.size();
        if (s_mins == s_maxs && s_maxs > 1 )
        {
            for (size_t s = 1; s < s_maxs; s++)
            {
                Range_t range;
                range.max = maxs.get(s).asDouble();
                range.min = mins.get(s).asDouble();
                if (range.max >= 0 && range.max <= 360 &&
                    range.min >= 0 && range.min <= 360 &&
                    range.max > range.min)
                {
                    range_skip_vector.push_back(range);
                    yDebug() << "Laser sensor: skipping angle between" << range.min << "and" << range.max;
                }
                else
                {
                    yError() << "Invalid range in SKIP section";
                    return false;
                }
            }
        }
    }
    return true;
}

bool GazeboYarpLaserSensorDriver::close()
{
    this->m_updateConnection.reset();

    return true;
}

//PRECISELY TIMED
yarp::os::Stamp GazeboYarpLaserSensorDriver::getLastInputStamp()
{
    return m_lastTimestamp;
}

bool GazeboYarpLaserSensorDriver::getRawData (yarp::sig::Vector &data)
{
    yarp::os::LockGuard guard(m_mutex);
    ///< \todo TODO in my opinion the reader should care of passing a vector of the proper dimension to the driver, but apparently this is not the case
    /*
    if( (int)m_forceTorqueData.size() != YarpForceTorqueChannelsNumber ||
        (int)out.size() != YarpForceTorqueChannelsNumber ) {
        return AS_ERROR;
    }
    */

   if (m_sensorData.size() != m_samples)
   {
       if (m_first_run)
       {
          m_device_status = DEVICE_TIMEOUT;
          return false;
       }
       else
       {
          m_device_status = DEVICE_GENERAL_ERROR;
          yError() << "Internal error";
          return false ;
       }
   }

   if (data.size() != m_samples)
   {
       data.resize(m_samples);
   }

    for (unsigned int i=0; i<m_samples; i++)
    {

      if (m_enable_discard_range)
      {
        if (m_sensorData[i]>=m_max_discard_range) m_sensorData[i]=INFINITY;
        if (m_sensorData[i]<=m_min_discard_range) m_sensorData[i]=INFINITY;
      }
      else if (m_enable_clip_range)
      {
        if (m_sensorData[i]>=m_max_clip_range) m_sensorData[i]=m_max_clip_range;
        if (m_sensorData[i]<=m_min_clip_range) m_sensorData[i]=m_min_clip_range;
      }

      double angle = i * m_resolution;
      for (size_t s = 0; s < range_skip_vector.size(); s++)
      {
        if (angle>=range_skip_vector[s].min && angle <= range_skip_vector[s].max)
        {
            m_sensorData[i] = INFINITY;
            //yDebug() <<"skipping" << s << angle << range_skip_vector[s].min << range_skip_vector[s].max;
        }
      }
      data[i]=m_sensorData[i];
    }

    m_device_status = DEVICE_OK_IN_USE;
	return true;
}

bool GazeboYarpLaserSensorDriver::getLaserMeasurement (std::vector<yarp::dev::LaserMeasurementData> &data)
{
    yarp::os::LockGuard guard(m_mutex);
    ///< \todo TODO in my opinion the reader should care of passing a vector of the proper dimension to the driver, but apparently this is not the case
    /*
    if( (int)m_forceTorqueData.size() != YarpForceTorqueChannelsNumber ||
        (int)out.size() != YarpForceTorqueChannelsNumber ) {
        return AS_ERROR;
    }
    */

   if (m_sensorData.size() != m_samples)
   {
       if (m_first_run)
       {
          m_device_status = DEVICE_TIMEOUT;
          return false;
       }
       else
       {
          m_device_status = DEVICE_GENERAL_ERROR;
          yError() << "Internal error";
          return false ;
       }
   }

   if (data.size() != m_samples)
   {
       data.resize(m_samples);
   }

    for (unsigned int i=0; i<m_samples; i++)
    {

      if (m_enable_discard_range)
      {
        if (m_sensorData[i]>=m_max_discard_range) m_sensorData[i]=INFINITY;
        if (m_sensorData[i]<=m_min_discard_range) m_sensorData[i]=INFINITY;
      }
      else if (m_enable_clip_range)
      {
        if (m_sensorData[i]>=m_max_clip_range) m_sensorData[i]=m_max_clip_range;
        if (m_sensorData[i]<=m_min_clip_range) m_sensorData[i]=m_min_clip_range;
      }

      double angle = i * m_resolution;
      for (size_t s = 0; s < range_skip_vector.size(); s++)
      {
        if (angle>=range_skip_vector[s].min && angle <= range_skip_vector[s].max)
        {
            m_sensorData[i] = INFINITY;
            //yDebug() <<"skipping" << s << angle << range_skip_vector[s].min << range_skip_vector[s].max;
        }
      }
      data[i].set_polar(m_sensorData[i],angle);
    }

    m_device_status = DEVICE_OK_IN_USE;
    return true;
}

bool GazeboYarpLaserSensorDriver::getDeviceStatus (Device_status &status)
{
    yarp::os::LockGuard guard(m_mutex);
    status = m_device_status;
    return true;
}

bool GazeboYarpLaserSensorDriver::getDistanceRange (double &min, double &max)
{
    yarp::os::LockGuard guard(m_mutex);
    min = m_min_gazebo_range;
    max = m_max_gazebo_range;
    return true;
}

bool GazeboYarpLaserSensorDriver::setDistanceRange (double min, double max)
{
    yarp::os::LockGuard guard(m_mutex);
    yError() << "setDistanceRange() Not yet implemented";
    return false;
}

bool GazeboYarpLaserSensorDriver::getScanLimits (double &min, double &max)
{
    yarp::os::LockGuard guard(m_mutex);
    min = m_min_angle;
    max = m_max_angle;
    return true;
}

bool GazeboYarpLaserSensorDriver::setScanLimits (double min, double max)
{
    yarp::os::LockGuard guard(m_mutex);
    yError() << "setScanLimits() Not yet implemented";
    return false;
}

bool GazeboYarpLaserSensorDriver::getHorizontalResolution (double &step)
{
    yarp::os::LockGuard guard(m_mutex);
    step = fabs(m_max_angle-m_min_angle)/m_samples;
    return true;
}

bool GazeboYarpLaserSensorDriver::setHorizontalResolution (double step)
{
    yarp::os::LockGuard guard(m_mutex);
    yError() << "setHorizontalResolution() Not yet implemented";
    return false;
}

bool GazeboYarpLaserSensorDriver::getScanRate (double &rate)
{
    yarp::os::LockGuard guard(m_mutex);
    rate = m_rate;
    return true;
}

bool GazeboYarpLaserSensorDriver::setScanRate (double rate)
{
    yarp::os::LockGuard guard(m_mutex);
    if (rate<0)
    {
      yError() << "Invalid setScanRate";
      return false;
    }
    m_parentSensor->SetUpdateRate(rate);
    m_rate = rate;
    return true;
}

bool GazeboYarpLaserSensorDriver::getDeviceInfo (std::string &device_info)
{
    yarp::os::LockGuard guard(m_mutex);
    return true;
}
