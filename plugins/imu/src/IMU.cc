/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "IMU.hh"
#include "IMUDriver.h"
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>


#include <gazebo/sensors/ImuSensor.hh>

#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

GZ_REGISTER_SENSOR_PLUGIN(gazebo::GazeboYarpIMU)

namespace gazebo {

GazeboYarpIMU::GazeboYarpIMU() : SensorPlugin()
{
}

GazeboYarpIMU::~GazeboYarpIMU()
{
    if(m_iWrap) {
        m_iWrap->detachAll();
        m_iWrap = nullptr;
    }
    if(m_iWrapAdditional) {
        m_iWrapAdditional->detachAll();
        m_iWrapAdditional = nullptr;
    }
    if(m_MASWrapper.isValid()) {
        m_MASWrapper.close();
    }
    if(m_AdditionalWrapper.isValid()) {
        m_AdditionalWrapper.close();
    }
    if(m_imuDriver.isValid()) {
        m_imuDriver.close();
    }
    GazeboYarpPlugins::Handler::getHandler()->removeSensor(m_scopedSensorName);
    yarp::os::Network::fini();
}

void GazeboYarpIMU::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    // The legacy behaviour is to support the old format of ini file with the device/subdevice structure
    bool legacy_behaviour{false};
    yarp::os::Network::init();
    if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
        yError() << "GazeboYarpIMU::Load error: yarp network does not seem to be available, is the yarpserver running?";
        return;
    }

    if (!_sensor) {
        gzerr << "GazeboYarpIMU plugin requires a IMUSensor." << std::endl;
        return;
    }

    _sensor->SetActive(true);

    // Add my gazebo device driver to the factory.
    ::yarp::dev::Drivers::factory().add(new ::yarp::dev::DriverCreatorOf< ::yarp::dev::GazeboYarpIMUDriver>
                                        ("gazebo_imu", "inertial", "GazeboYarpIMUDriver"));

    bool configuration_loaded = GazeboYarpPlugins::loadConfigSensorPlugin(_sensor,_sdf,m_parameters);

    if (!configuration_loaded) {
        return;
    }
    m_scopedSensorName = _sensor->ScopedName();

    //Insert the pointer in the singleton handler for retriving it in the yarp driver
    GazeboYarpPlugins::Handler::getHandler()->setSensor(_sensor.get());

    /*
    * Open the driver wrapper
    */
    //Retrieve wrapper properties
    ::yarp::os::Bottle MASwrapperProp = m_parameters.findGroup("WRAPPER");
    if(MASwrapperProp.isNull())
    {
        yWarning("GazeboYarpIMU : [WRAPPER] group not found in config file, maybe you are using the old version of the ini file, please update icub-gazebo\n");
        yWarning("GazeboYarpIMU : trying to open it with the legacy behaviour device-subdevice");
        legacy_behaviour = true;
    }

    if (legacy_behaviour) {
        m_parameters.put(YarpIMUScopedName.c_str(), m_scopedSensorName.c_str());
        m_parameters.put("sensor_name",_sensor->Name());
        if (!m_imuDriver.open(m_parameters)) {
            yError() << "GazeboYarpIMU Plugin Load failed: error in opening yarp driver";
        }
        return;
    }

    //Open the driver wrapper
    if (!m_MASWrapper.open(MASwrapperProp))
    {
        yError() << "GazeboYarpIMU Plugin Load failed: error in opening the yarp wrapper";
    }

    /*
    * Open the old wrapper
    */
    //Retrieve wrapper properties
    ::yarp::os::Bottle AdditionalWrapperProp = m_parameters.findGroup("ADDITIONAL_WRAPPER");
    if(AdditionalWrapperProp.isNull())
    {
        yError("GazeboYarpIMU : [ADDITIONAL_WRAPPER] group not found in config file\n");
        return;
    }

    //Open the driver wrapper
    if (!m_AdditionalWrapper.open(AdditionalWrapperProp))
    {
        yError() << "GazeboYarpIMU Plugin Load failed: error in opening the yarp wrapper";
    }


    /*
     * Open the imu driver
     */
    //Retrieve part driver properties
    ::yarp::os::Bottle imu_properties = m_parameters.findGroup("IMU_DRIVER");
    if(imu_properties.isNull())
    {
        yError("GazeboYarpIMU : [IMU_DRIVER] group not found in config file\n");
        return;
    }

    //Add the model scoped name for later retrieval of the child sensors from the Handler
    yarp::os::Bottle& robotNameProp = imu_properties.addList();
    robotNameProp.addString(YarpIMUScopedName);
    robotNameProp.addString(m_scopedSensorName);

    imu_properties.addString("sensor_name");
    imu_properties.addString(_sensor->Name());

    //Open the driver
    if (m_imuDriver.open(imu_properties)) {
    } else {
        yError() << "GazeboYarpIMU Plugin Load failed: error in opening yarp driver";
    }

    //Attach the part driver to the wrapper
    if(!m_MASWrapper.view(m_iWrap) || (!m_AdditionalWrapper.view(m_iWrapAdditional))){
        yError()<< "GazeboYarpIMU Plugin Load failed: unable to view iMultipleWrapper interfaces";
        return;
    }
    ::yarp::dev::PolyDriverList driverList;
    driverList.push(&m_imuDriver,"dummy");
    if(!m_iWrap->attachAll(driverList) || ! m_iWrapAdditional->attachAll(driverList))
    {
        yError() << "GazeboYarpIMU: error in connecting wrapper and device ";
    }
}

}
