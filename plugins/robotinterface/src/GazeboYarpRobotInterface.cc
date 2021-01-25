/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "GazeboYarpRobotInterface.hh"
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>

#include <gazebo/physics/Model.hh>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>

namespace gazebo
{

GazeboYarpRobotInterface::GazeboYarpRobotInterface()
{
    std::cerr << "GazeboYarpRobotInterface constructor" << std::endl;
}

GazeboYarpRobotInterface::~GazeboYarpRobotInterface()
{
    // Close robotinterface 
    bool ok = m_xmlRobotInterfaceResult.robot.enterPhase(yarp::robotinterface::experimental::ActionPhaseInterrupt1);
    if (!ok) {
        yError() << "GazeboYarpRobotInterface: impossible to run phase ActionPhaseInterrupt1 robotinterface";
    }
    ok = m_xmlRobotInterfaceResult.robot.enterPhase(yarp::robotinterface::experimental::ActionPhaseShutdown);
    if (!ok) {
        yError() << "GazeboYarpRobotInterface: impossible  to run phase ActionPhaseShutdown in robotinterface";
    }

    yarp::os::Network::fini();
}

void GazeboYarpRobotInterface::Load(physics::ModelPtr _parentModel, sdf::ElementPtr _sdf)
{
    yarp::os::Network::init();

    if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
        yError() << "GazeboYarpRobotInterface : yarp network does not seem to be available, is the yarpserver running?";
        return;
    }

    if (!_parentModel) {
        gzerr << "GazeboYarpRobotInterface plugin requires a parent.\n";
        return;
    }

    GazeboYarpPlugins::Handler::getHandler()->setRobot(get_pointer(_parentModel));

    // Getting .xml and loading configuration file from sdf
    bool loaded_configuration = false;
    if (_sdf->HasElement("yarpRobotInterfaceConfigurationFile"))
    {
        std::string robotinterface_file_name = _sdf->Get<std::string>("yarpRobotInterfaceConfigurationFile");
        std::string robotinterface_file_path = gazebo::common::SystemPaths::Instance()->FindFileURI(robotinterface_file_name);

        if (robotinterface_file_name == "") {
            yError() << "GazeboYarpRobotInterface error: failure in finding robotinterface configuration for model" << _parentModel->GetName() << "\n"
                      << "GazeboYarpRobotInterface error: yarpRobotInterfaceConfigurationFile : " << robotinterface_file_name << "\n"
                      << "GazeboYarpRobotInterface error: yarpRobotInterfaceConfigurationFile absolute path : " << robotinterface_file_path;
            loaded_configuration = false;
        } else {
            m_xmlRobotInterfaceResult = m_xmlRobotInterfaceReader.getRobotFromFile(robotinterface_file_path);

            if (m_xmlRobotInterfaceResult.parsingIsSuccessful) {
                loaded_configuration = true;
            } else {
                yError() << "GazeboYarpRobotInterface error: failure in loading robotinterface configuration for model" << _parentModel->GetName() << "\n"
                      << "GazeboYarpRobotInterface error: yarpRobotInterfaceConfigurationFile : " << robotinterface_file_name << "\n"
                      << "GazeboYarpRobotInterface error: yarpRobotInterfaceConfigurationFile absolute path : " << robotinterface_file_path;
                loaded_configuration = false; 
            }
        }
    }

    if (!loaded_configuration) {
        yError() << "GazeboYarpRobotInterface : xml file specified in yarpRobotInterfaceConfigurationFile not found or not loaded.";
        return;
    }

    // Extract externalDriverList of  devices from the one that have been already opened in the Gazebo model by other gazebo_yarp plugins 
    yarp::dev::PolyDriverList externalDriverList;
    GazeboYarpPlugins::Handler::getHandler()->getDevicesAsPolyDriverList(externalDriverList);

    // Set external devices from the one that have been already opened in the Gazebo model by other gazebo_yarp plugins 
    bool ok = m_xmlRobotInterfaceResult.robot.setExternalDevices(externalDriverList);
    if (!ok) {
        yError() << "GazeboYarpRobotInterface : impossible to set external devices";
    }

    // Start robotinterface 
    ok = m_xmlRobotInterfaceResult.robot.enterPhase(yarp::robotinterface::experimental::ActionPhaseStartup);
    if (!ok) {
        yError() << "GazeboYarpRobotInterface : impossible to start robotinterface";
    }
}


// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboYarpRobotInterface)
}
