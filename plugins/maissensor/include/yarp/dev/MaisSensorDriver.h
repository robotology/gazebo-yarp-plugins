/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_MAISSENSORDRIVER_HH
#define GAZEBOYARP_MAISSENSORDRIVER_HH

#include <yarp/os/Property.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Mutex.h>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <gazebo/math/Angle.hh>

namespace yarp {
    namespace dev {
        class GazeboYarpMaisSensorDriver;
    }
}

namespace gazebo {
    namespace common {
        class UpdateInfo;
    }

    namespace physics {
        class Model;
        class Joint;
        typedef boost::shared_ptr<Joint> JointPtr;
    }

    namespace event {
        class Connection;
        typedef boost::shared_ptr<Connection> ConnectionPtr;
    }

    namespace transport {
        class Node;
        class Publisher;

        typedef boost::shared_ptr<Node> NodePtr;
        typedef boost::shared_ptr<Publisher> PublisherPtr;
    }

    namespace msgs {
        class JointCmd;
    }
}

class yarp::dev::GazeboYarpMaisSensorDriver:
    public DeviceDriver,
    public DeviceResponder,
    public IAnalogSensor
{
public:

    GazeboYarpMaisSensorDriver();
    virtual ~GazeboYarpMaisSensorDriver();

    /**
     * Gazebo stuff
     */
    bool gazebo_init();
    void onUpdate(const gazebo::common::UpdateInfo&);

    /**
     * Yarp interfaces start here
     */


    //DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();


private:

    enum JointType
    {
        JointType_Unknown = 0,
        JointType_Revolute,
        JointType_Prismatic
    };
    
    std::string deviceName;
    gazebo::physics::Model* m_robot;
    gazebo::event::ConnectionPtr m_updateConnection;



    yarp::os::Property m_pluginParameters; /**< Contains the parameters of the device contained in the yarpConfigurationFile .ini file */

    yarp::sig::Vector m_positions; /**< joint positions [Degrees] */
    unsigned int m_numberOfJoints; /**< number of joints controlled by the control board */


    yarp::os::Stamp m_lastTimestamp; /**< timestamp, updated with simulation time at each onUpdate call */

    yarp::os::Mutex m_mutex;
    yarp::sig::VectorOf<JointType> m_jointTypes;

    int m_channels_num;
    std::vector<std::string> m_jointNames;
    std::vector<std::string> controlboard_joint_names;
    std::vector<gazebo::physics::JointPtr> m_jointPointers; /* pointers for each joint, avoiding several calls to getJoint(joint_name) */
    gazebo::transport::NodePtr m_gazeboNode;


    bool started;
    int m_clock;
    int _T_controller;

    /**
     * Private methods
     */
    bool configureJointType();
    bool setJointNames();  //WORKS

    virtual int read(yarp::sig::Vector &out);
    virtual int getState(int ch);
    virtual int getChannels();
    virtual int calibrateSensor();
    virtual int calibrateSensor(const yarp::sig::Vector& value);
    virtual int calibrateChannel(int ch);
    virtual int calibrateChannel(int ch, double value);
    
    /**
     * \brief convert data read from Gazebo to user unit sistem,
     *  e.g. degrees for revolute joints and meters for prismatic joints
     * \param joint joint number
     * \param value Raw value read from Gazebo function like 'GetAngle'
     * \return value in user units
     */
    double convertGazeboToUser(int joint, gazebo::math::Angle value);
    double convertGazeboToUser(int joint, double value);
    double *convertGazeboToUser(double *values);

    /**
     * \brief convert data read from user unit sistem to Gazebo one
     *  e.g. radiants for revolute joints and meters for prismatic joints
     * \param joint joint number
     * \param value Raw value read from Gazebo function like 'GetAngle'
     * \return value in Gazebo units (SI)
     */
    double convertUserToGazebo(int joint, double value);
    double convertUserToGazebo(double value);
    double *convertUserToGazebo(double *values);
};

#endif //GAZEBOYARP_MAISSENSORDRIVER_HH
