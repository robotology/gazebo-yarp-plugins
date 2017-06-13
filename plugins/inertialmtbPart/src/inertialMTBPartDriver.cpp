/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "inertialMTBPartDriver.h"
#include <GazeboYarpPlugins/ConfHelpers.hh>
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>

#include <gazebo/math/Vector3.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace yarp::dev;

const std::string MTBPartDriverParentScopedName = "ModelScopedName";

std::map<std::string,int> GazeboYarpInertialMTBPartDriver::LUTpart2maxSensors{
    {"left_arm",8},
    {"right_arm",8},
    {"left_leg",13},
    {"right_leg",13},
    {"torso",4}
};

const double GazeboYarpInertialMTBPartDriver::version = 6.0;

const std::string GazeboYarpInertialMTBPartDriver::LUTmtbPosEnum2Id[1+eoas_inertial_pos_offsetcentral+4] = {"none",
    "1b7","1b8","1b9","1b10","1b11","1b12","1b13",
    "10b12","10b13","10b8","10b9","10b10","10b11","10b1","10b2","10b3","10b4","10b5","10b6","10b7",
    "none","none","none","none",
    "2b7","2b8","2b9","2b10","2b11","2b12","2b13",
    "11b12","11b13","11b8","11b9","11b10","11b11","11b1","11b2","11b3","11b5","11b4","11b6","11b7",
    "none","none","none","none",
    "9b7","9b8","9b9","9b10"};

std::map<std::string,int> GazeboYarpInertialMTBPartDriver::generateLUTmtbId2PosEnum()
{
    // Fill the mapping from MTB sensor labels to position Ids
    std::map<std::string,int> lut;
    for (int posId = 0; posId < sizeof(LUTmtbPosEnum2Id)/sizeof(std::string); posId++)
    {
        lut[LUTmtbPosEnum2Id[posId]] = posId;
    }
    return lut;
}

std::map<std::string,int> GazeboYarpInertialMTBPartDriver::LUTmtbId2PosEnum = generateLUTmtbId2PosEnum();

std::map<std::string,int> GazeboYarpInertialMTBPartDriver::LUTmtbType2enum{
    {"none",eoas_inertial_type_none},
    {"acc",eoas_inertial_type_accelerometer},
    {"gyro",eoas_inertial_type_gyroscope}
};


GazeboYarpInertialMTBPartDriver::GazeboYarpInertialMTBPartDriver() {}
GazeboYarpInertialMTBPartDriver::~GazeboYarpInertialMTBPartDriver() {}

/**
 *
 * Export an inertial sensor.
 * The 3 channels are non calibrated 3-axis (X, Y, Z) acceleration data.
 *
 */
void GazeboYarpInertialMTBPartDriver::onUpdate(const gazebo::common::UpdateInfo &updateInfo)
{
    // Get current Gazebo timestamp (ms). This is a global timestamp.
    // The individual timestamps of each sensor might slightly differ
    // as it is the case on the real robot.
    // This timestamp can be accessed by the wrapper via the IPreciselyTimed
    // interface. Instead, the Analog wrapper is using the its own stamp
    // along with the Yarp clock. If Gazebo is launched with the clock plugin,
    // the MTB driver and the wrapper will have the same timestamp.
    m_lastGazeboTimestamp.update(updateInfo.simTime.Double());

    // Get all sensors data
    std::vector<gazebo::sensors::ImuSensor*>::iterator sensorIter;
    int bufferOffset;
    for (sensorIter = m_enabledSensors.begin(),
         bufferOffset = sensorDataStartOffset+sensorTimestpNmeasOffset;
         sensorIter < m_enabledSensors.end();
         sensorIter++,
         bufferOffset+=sensorDataLength)
    {
#if GAZEBO_MAJOR_VERSION >= 6
        ignition::math::Vector3d linear_acceleration = (*sensorIter)->LinearAcceleration();
#else
        gazebo::math::Vector3 linear_acceleration = (*sensorIter)->GetLinearAcceleration();
#endif

#if GAZEBO_MAJOR_VERSION >= 7
        double sensorLastTimestamp = (*sensorIter)->LastUpdateTime().Double();
#else
        double sensorLastTimestamp = (*sensorIter)->GetLastUpdateTime().Double();
#endif

        //Fill the 3 channels measurement data
        m_dataMutex.wait();
        m_inertialmtbOutBuffer[bufferOffset] = sensorLastTimestamp;
        for (unsigned idx = 0; idx < 3; idx++) {
            m_inertialmtbOutBuffer[bufferOffset+1+idx] = linear_acceleration[idx];
        }
        m_dataMutex.post();
    }
}

//DEVICE DRIVER
bool GazeboYarpInertialMTBPartDriver::open(yarp::os::Searchable& config)
{
    // Get the parent model (robot scoped name) for rebuilding the
    // sensors scoped names
    std::string robotScopedName(config.find(MTBPartDriverParentScopedName.c_str()).asString().c_str());
    // Get the list of enabled sensors
    yarp::os::Bottle enabledSensors = *config.find("enabledSensors").asList();
    // Build the vector of enabled sensors pointers
    buildEnabledSensorsVector(robotScopedName,enabledSensors);

    // Get the part to which the sensors are attached
    std::string robotPart = config.find("robotPart").asString().c_str();
    // Build the fixed parameters in the output buffer "m_inertialmtbOutBuffer"
    buildOutBufferFixedData(robotPart,enabledSensors);

    //Connect the driver to the gazebo simulation
    m_updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboYarpInertialMTBPartDriver::onUpdate, this, _1));

    return true;
}

bool GazeboYarpInertialMTBPartDriver::close()
{
    if (m_updateConnection.get()) {
        gazebo::event::Events::DisconnectWorldUpdateBegin(m_updateConnection);
        m_updateConnection = gazebo::event::ConnectionPtr();
    }
    return true;
}

//GENERIC SENSOR
int GazeboYarpInertialMTBPartDriver::read(yarp::sig::Vector &out)
{
    if (out.size() != m_inertialmtbOutBuffer.size()) {
        yError() << "GazeboYarpInertialMTBPartDriver Error: output buffer to wrapper size mismatch!";
        return AS_ERROR;
    }

    m_dataMutex.wait();
    out = m_inertialmtbOutBuffer;
    m_dataMutex.post();

    return AS_OK;
}

int GazeboYarpInertialMTBPartDriver::getChannels()
{
    return m_nbChannels;
}

int GazeboYarpInertialMTBPartDriver::getState(int /*ch*/)
{
    return AS_OK;
}

int GazeboYarpInertialMTBPartDriver::calibrateSensor()
{
    return AS_OK;
}

int GazeboYarpInertialMTBPartDriver::calibrateSensor(const yarp::sig::Vector& /*value*/)
{
    return AS_OK;
}

int GazeboYarpInertialMTBPartDriver::calibrateChannel(int /*ch*/)
{
    return AS_OK;
}

int GazeboYarpInertialMTBPartDriver::calibrateChannel(int /*ch*/, double /*v*/)
{
    return AS_OK;
}

yarp::os::Stamp GazeboYarpInertialMTBPartDriver::getLastInputStamp()
{
    return m_lastGazeboTimestamp;
}

bool GazeboYarpInertialMTBPartDriver::buildEnabledSensorsVector(std::string robotScopedName,
                                                                yarp::os::Bottle & enabledSensors)
{
    // resize the sensors vector
    m_enabledSensors.resize(enabledSensors.size());

    // Go through the sensors listed in 'enabledSensors'
    for (int sensorIter = 0; sensorIter < enabledSensors.size(); sensorIter++)
    {
        // get the next sensor scoped name from the input list
        std::string sensorScopedName = robotScopedName + "::" + enabledSensors.get(sensorIter).asString();

        //Get the gazebo pointer (we assume it has been previously added by a sensor plugin)
        gazebo::sensors::ImuSensor* sensor = dynamic_cast<gazebo::sensors::ImuSensor*>(GazeboYarpPlugins::Handler::getHandler()->getSensor(sensorScopedName));

        if (!sensor) {
            yError() << "GazeboYarpInertialMTBPartDriver Error: required sensor " << sensorScopedName << " was not found";
            return false;
        }

        // Add sensor to the building vector
        m_enabledSensors[sensorIter] = sensor;
    }

    return true;
}

bool GazeboYarpInertialMTBPartDriver::buildOutBufferFixedData(std::string robotPart,
                                                              yarp::os::Bottle & enabledSensors)
{
    // Resize the output buffer
    m_nbChannels = sensorDataStartOffset + LUTpart2maxSensors[robotPart]*sensorDataLength;
    m_inertialmtbOutBuffer.resize(m_nbChannels,0);

    /*
     * Go through the buffer and fill the metadata
     */
    m_dataMutex.wait();

    // number of enabled sensors and VERsion of the format
    m_inertialmtbOutBuffer(0) = double(enabledSensors.size());
    m_inertialmtbOutBuffer(1) = version;

    // got through all enabled sensors
    for (int sensorIdx = 0, bufferOffset = sensorDataStartOffset;
         sensorIdx < enabledSensors.size();
         sensorIdx++, bufferOffset+=sensorDataLength)
    {
        std::vector<std::string> explodedSensorName =
        GazeboYarpPlugins::splitString(enabledSensors.get(sensorIdx).asString(),"_");
        // Get the sensor label (1b1, 1b2, ...)
        std::string sensorLabel = *(explodedSensorName.end()-1);
        // Get the sensor type (accelerometer or gyroscope)
        std::string sensorType = *(explodedSensorName.end()-2);
        // Set the metadata in the output buffer
        m_inertialmtbOutBuffer(bufferOffset+sensorIdxOffset) = double(LUTmtbId2PosEnum[sensorLabel]);
        m_inertialmtbOutBuffer(bufferOffset+sensorTypeOffset) = double(LUTmtbType2enum[sensorType]);
    }

    // The remaining content is already by default set to zero

    m_dataMutex.post();

    return true;
}
