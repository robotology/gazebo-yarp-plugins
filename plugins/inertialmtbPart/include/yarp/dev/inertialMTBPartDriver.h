/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_INERTIALMTBPARTDRIVER_H
#define GAZEBOYARP_INERTIALMTBPARTDRIVER_H

#include <vector>
#include <map>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/Semaphore.h>

#include <boost/shared_ptr.hpp>

//Forward declarations
namespace yarp {
    namespace dev {
        class GazeboYarpInertialMTBPartDriver;
    }
}

namespace gazebo {
    namespace common {
        class UpdateInfo;
    }
    namespace sensors {
        class ImuSensor;
    }
    namespace event {
        class Connection;
        typedef boost::shared_ptr<Connection> ConnectionPtr;
    }
}

extern const unsigned YarpInertialMTBChannelsNumber; // Each MTB inertial sensor has 3 fixed channels
extern const std::string MTBPartDriverParentScopedName;

/// \class GazeboYarpInertialMTBPartDriver
///
/// This class implements the driver that exposes the sensors of a whole
/// model part (ex: the leg) on the yarp network. A part can include
/// several links (ex: upper_leg, lower_leg and foot).
/// It can be configurated using the yarpConfigurationFile sdf tag,
/// that contains a Gazebo URI pointing at a yarp .ini configuration file.
class yarp::dev::GazeboYarpInertialMTBPartDriver:
    public yarp::dev::IAnalogSensor,
    public yarp::dev::IPreciselyTimed,
    public yarp::dev::DeviceDriver
{
public:
    GazeboYarpInertialMTBPartDriver();

    virtual ~GazeboYarpInertialMTBPartDriver();

    void onUpdate(const gazebo::common::UpdateInfo&);

    /**
     * Yarp interfaces start here
     */

    //DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    //ANALOG SENSOR
    virtual int read(yarp::sig::Vector& out);
    virtual int getState(int channel);
    virtual int getChannels();
    virtual int calibrateChannel(int channel, double v);
    virtual int calibrateSensor();
    virtual int calibrateSensor(const yarp::sig::Vector& value);
    virtual int calibrateChannel(int channel);

    //PRECISELY TIMED
    virtual yarp::os::Stamp getLastInputStamp();


private:
    /**
     *
     * Build the vector of enabled sensors as per the ordered
     * list retrieved from the part ini configuration file
     */
    inline bool buildEnabledSensorsVector(std::string robotScopedName,
                                          yarp::os::Bottle & enabledSensors);

    /**
     *
     * Fill the output buffer with the inertial sensors fixed metadata as per
     * EMS data format specified further below: n, VER, ai, bi.
     */
    inline bool buildOutBufferFixedData(std::string robotPart,
                                        yarp::os::Bottle & enabledSensors);

    //Yarp interface parameters
    yarp::sig::Vector m_inertialmtbOutBuffer; //buffer for the exported data
    yarp::os::Stamp m_lastGazeboTimestamp; //buffer for last timestamp data
    int m_nbChannels; //depends on the part to which the sensors
                      //are attached. This size is fixed. If a
                      //sensor is deactivated, the respective slot
                      //is set to all zeros.

    //Inner parameters
    yarp::os::Semaphore m_dataMutex; //mutex for accessing the data
    std::vector<gazebo::sensors::ImuSensor*> m_enabledSensors;
    static std::map<std::string,int> LUTpart2maxSensors;

    //Connection and synchro with Gazebo
    gazebo::event::ConnectionPtr m_updateConnection;

    /**
     *
     * ==== EMS Data format ====:
     *
     * The network interface is a single Port for a whole limb/part
     * (left_leg, left_arm, rught_arm, torso, ...)
     * We will stream bottles with n accelerometers measurements:
     *
     * [n  VER  (a1 b1 t1 x1 y1 z1)   .... (an bn tn xn yn zn)]
     * n  = number of sensors
     * VER= current format version: 6.0
     * ai = pos of sensor ... see enum type
     * bi = accel (1) or gyro (2)
     * ti = time stamp.
     * xi,yi,zi = the 3 measurement channels of the accelerometer (non calibrated)
     *
     */
    static const double version;
    static const int sensorDataLength = 6;
    static const int sensorDataStartOffset = 2;
    static const int sensorTypeOffset = 1;
    static const int sensorIdxOffset = 0;
    static const int sensorTimestpNmeasOffset = 2;

    // Below enums are directly copied from icub-firmware-shared/eth/embobj/plus/comm-v2/icub/EoAnalogSensors.h

    // Matches sensorTypeT
    typedef enum
    {
        eoas_inertial_type_none          = 0,
        eoas_inertial_type_accelerometer = 1,
        eoas_inertial_type_gyroscope     = 2
    } eOas_inertial_type_t;

    enum { eoas_inertial_pos_max_numberof = 63 };

    enum { eoas_inertial_pos_offsetleft = 0, eoas_inertial_pos_offsetright = 24, eoas_inertial_pos_offsetcentral = 48 };

    /** @typedef    typedef enum eOas_inertial_position_t
     @brief      contains a unique id for every possible inertial sensor positioned on iCub. So far we can host
     up to 63 different positions. The actual positions on iCub are documented on http://wiki.icub.org/wiki/Distributed_Inertial_sensing
     where one must look for the tags 10B12, 10B13 etc. The mapping on CAN for the ETH robot v3 is written aside.
     **/
    typedef enum
    {
        eoas_inertial_pos_none                  = 0,

        // left arm
        eoas_inertial_pos_l_hand                = 1+eoas_inertial_pos_offsetleft,       // label 1B7    canloc = (CAN2, 14)
        eoas_inertial_pos_l_forearm_1           = 2+eoas_inertial_pos_offsetleft,       // label 1B8    canloc = (CAN2, 12)
        eoas_inertial_pos_l_forearm_2           = 3+eoas_inertial_pos_offsetleft,       // label 1B9    canloc = (CAN2, 13)
        eoas_inertial_pos_l_upper_arm_1         = 4+eoas_inertial_pos_offsetleft,       // label 1B10   canloc = (CAN2,  9)
        eoas_inertial_pos_l_upper_arm_2         = 5+eoas_inertial_pos_offsetleft,       // label 1B11   canloc = (CAN2, 11)
        eoas_inertial_pos_l_upper_arm_3         = 6+eoas_inertial_pos_offsetleft,       // label 1B12   canloc = (CAN2, 10)
        eoas_inertial_pos_l_upper_arm_4         = 7+eoas_inertial_pos_offsetleft,       // label 1B13   canloc = (CAN2,  8)
        // left leg
        eoas_inertial_pos_l_foot_1              = 8+eoas_inertial_pos_offsetleft,       // label 10B12  canloc = (CAN2, 13)
        eoas_inertial_pos_l_foot_2              = 9+eoas_inertial_pos_offsetleft,       // label 10B13  canloc = (CAN2, 12)
        eoas_inertial_pos_l_lower_leg_1         = 10+eoas_inertial_pos_offsetleft,      // label 10B8   canloc = (CAN2,  8)
        eoas_inertial_pos_l_lower_leg_2         = 11+eoas_inertial_pos_offsetleft,      // label 10B9   canloc = (CAN2,  9)
        eoas_inertial_pos_l_lower_leg_3         = 12+eoas_inertial_pos_offsetleft,      // label 10B10  canloc = (CAN2, 10)
        eoas_inertial_pos_l_lower_leg_4         = 13+eoas_inertial_pos_offsetleft,      // label 10B11  canloc = (CAN2, 11)
        eoas_inertial_pos_l_upper_leg_1         = 14+eoas_inertial_pos_offsetleft,      // label 10B1   canloc = (CAN1,  1)
        eoas_inertial_pos_l_upper_leg_2         = 15+eoas_inertial_pos_offsetleft,      // label 10B2   canloc = (CAN1,  2)
        eoas_inertial_pos_l_upper_leg_3         = 16+eoas_inertial_pos_offsetleft,      // label 10B3   canloc = (CAN1,  3)
        eoas_inertial_pos_l_upper_leg_4         = 17+eoas_inertial_pos_offsetleft,      // label 10B4   canloc = (CAN1,  4)
        eoas_inertial_pos_l_upper_leg_5         = 18+eoas_inertial_pos_offsetleft,      // label 10B5   canloc = (CAN1,  5)
        eoas_inertial_pos_l_upper_leg_6         = 19+eoas_inertial_pos_offsetleft,      // label 10B6   canloc = (CAN1,  6)
        eoas_inertial_pos_l_upper_leg_7         = 20+eoas_inertial_pos_offsetleft,      // label 10B7   canloc = (CAN1,  7)

        // right arm
        eoas_inertial_pos_r_hand                = 1+eoas_inertial_pos_offsetright,      // label 2B7    canloc = (CAN2, 14)
        eoas_inertial_pos_r_forearm_1           = 2+eoas_inertial_pos_offsetright,      // label 2B8    canloc = (CAN2, 12)
        eoas_inertial_pos_r_forearm_2           = 3+eoas_inertial_pos_offsetright,      // label 2B9    canloc = (CAN2, 13)
        eoas_inertial_pos_r_upper_arm_1         = 4+eoas_inertial_pos_offsetright,      // label 2B10   canloc = (CAN2,  9)
        eoas_inertial_pos_r_upper_arm_2         = 5+eoas_inertial_pos_offsetright,      // label 2B11   canloc = (CAN2, 11)
        eoas_inertial_pos_r_upper_arm_3         = 6+eoas_inertial_pos_offsetright,      // label 2B12   canloc = (CAN2, 10)
        eoas_inertial_pos_r_upper_arm_4         = 7+eoas_inertial_pos_offsetright,      // label 2B13   canloc = (CAN2,  8)
        // right leg
        eoas_inertial_pos_r_foot_1              = 8+eoas_inertial_pos_offsetright,      // label 11B12  canloc = (CAN2, 13)
        eoas_inertial_pos_r_foot_2              = 9+eoas_inertial_pos_offsetright,      // label 11B13  canloc = (CAN2, 12)
        eoas_inertial_pos_r_lower_leg_1         = 10+eoas_inertial_pos_offsetright,     // label 11B8   canloc = (CAN2,  8)
        eoas_inertial_pos_r_lower_leg_2         = 11+eoas_inertial_pos_offsetright,     // label 11B9   canloc = (CAN2,  9)
        eoas_inertial_pos_r_lower_leg_3         = 12+eoas_inertial_pos_offsetright,     // label 11B10  canloc = (CAN2, 10)
        eoas_inertial_pos_r_lower_leg_4         = 13+eoas_inertial_pos_offsetright,     // label 11B11  canloc = (CAN2, 11)
        eoas_inertial_pos_r_upper_leg_1         = 14+eoas_inertial_pos_offsetright,     // label 11B1   canloc = (CAN1,  1)
        eoas_inertial_pos_r_upper_leg_2         = 15+eoas_inertial_pos_offsetright,     // label 11B2   canloc = (CAN1,  2)
        eoas_inertial_pos_r_upper_leg_3         = 16+eoas_inertial_pos_offsetright,     // label 11B3   canloc = (CAN1,  3)
        eoas_inertial_pos_r_upper_leg_4         = 17+eoas_inertial_pos_offsetright,     // label 11B5   canloc = (CAN1,  5)
        eoas_inertial_pos_r_upper_leg_5         = 18+eoas_inertial_pos_offsetright,     // label 11B4   canloc = (CAN1,  4)
        eoas_inertial_pos_r_upper_leg_6         = 19+eoas_inertial_pos_offsetright,     // label 11B6   canloc = (CAN1,  6)
        eoas_inertial_pos_r_upper_leg_7         = 20+eoas_inertial_pos_offsetright,     // label 11B7   canloc = (CAN1,  7)

        // central parts
        eoas_inertial_pos_chest_1               = 1+eoas_inertial_pos_offsetcentral,    // 9B7
        eoas_inertial_pos_chest_2               = 2+eoas_inertial_pos_offsetcentral,    // 9B8
        eoas_inertial_pos_chest_3               = 3+eoas_inertial_pos_offsetcentral,    // 9B9
        eoas_inertial_pos_chest_4               = 4+eoas_inertial_pos_offsetcentral,    // 9B10

        eOas_inertial_pos_jolly_1               = 60,
        eOas_inertial_pos_jolly_2               = 61,
        eOas_inertial_pos_jolly_3               = 62,
        eOas_inertial_pos_jolly_4               = 63

    } eOas_inertial_position_t;

    // LUT of MTB IDs (LUT output) indexed by the MTB enum defined above (LUT input).
    static const std::string LUTmtbPosEnum2Id[1+eoas_inertial_pos_offsetcentral+4];

    /**
     *
     * Generate the mapping from MTB sensor labels to position Ids (there is a
     * unique id for every possible inertial sensor positioned on iCub)
     */
    static std::map<std::string,int> generateLUTmtbId2PosEnum();

    // LUT mapping from MTB sensor labels (input) to position Ids (output)
    static std::map<std::string,int> LUTmtbId2PosEnum;

    // LUT mapping from MTB sensor type (input) to a type Id (output)
    static std::map<std::string,int> LUTmtbType2enum;
};

#endif // GAZEBOYARP_INERTIALMTBPARTDRIVER_H
