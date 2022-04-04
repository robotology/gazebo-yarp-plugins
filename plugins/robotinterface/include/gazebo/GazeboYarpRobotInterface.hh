/*
 * Copyright (C) Fondazione Istituto Italiano di Tecnologia 
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_ROBOTINTERFACEPLUGIN_HH
#define GAZEBOYARP_ROBOTINTERFACEPLUGIN_HH

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>

#include <yarp/robotinterface/XMLReader.h>

namespace gazebo
{
    class GazeboYarpRobotInterface;
}

/**
 * See gazebo-yarp-plugins/plugins/robotinterface/README.md for documentation on this plugin.
 *
 */
class gazebo::GazeboYarpRobotInterface : public gazebo::ModelPlugin
{
public:
    GazeboYarpRobotInterface();
    virtual ~GazeboYarpRobotInterface();
    
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    void CloseRobotInterface();
    void OnDeviceCompletlyRemoved(std::string scopedDeviceName);

private:
    yarp::robotinterface::XMLReader m_xmlRobotInterfaceReader;
    yarp::robotinterface::XMLReaderResult m_xmlRobotInterfaceResult;
    std::vector<std::string> m_deviceScopedNames;
    gazebo::event::ConnectionPtr m_connection;
    bool m_robotInterfaceCorrectlyStarted;
};


#endif
