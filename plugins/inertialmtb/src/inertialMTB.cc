/*
 * Copyright (C) 2013-2017 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "inertialMTB.hh"
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <gazebo/sensors/ImuSensor.hh>

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

GZ_REGISTER_SENSOR_PLUGIN(gazebo::GazeboYarpInertialMTB)

namespace gazebo {

GazeboYarpInertialMTB::GazeboYarpInertialMTB() : SensorPlugin()
{
}

GazeboYarpInertialMTB::~GazeboYarpInertialMTB()
{
    GazeboYarpPlugins::Handler::getHandler()->removeSensor(this->m_sensorName);
}

void GazeboYarpInertialMTB::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    if (!_sensor) {
        gzerr << "GazeboYarpInertialMTB plugin requires a IMUSensor." << std::endl;
        return;
    }

    _sensor->SetActive(true);

#if GAZEBO_MAJOR_VERSION >= 7
    m_sensorName = _sensor->ScopedName();
#else
    m_sensorName = _sensor->GetScopedName();
#endif
    //Insert the pointer in the singleton handler for retriving it in the yarp driver
    GazeboYarpPlugins::Handler::getHandler()->setSensor(_sensor.get());
}

}
