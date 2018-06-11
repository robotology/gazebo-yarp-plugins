/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "FakeControlBoard.hh"
#include "FakeControlBoardDriver.h"
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <gazebo/physics/Model.hh>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace std;
namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN(GazeboYarpFakeControlBoard)

    GazeboYarpFakeControlBoard::GazeboYarpFakeControlBoard() : m_iWrap(0)
    {}

    GazeboYarpFakeControlBoard::~GazeboYarpFakeControlBoard()
    {
        if (m_iWrap) {
            m_iWrap->detachAll();
            m_iWrap = 0;
        }

        if (m_wrapper.isValid()) {
            m_wrapper.close();
        }

        for (int n = 0; n < m_controlBoards.size(); n++) {
            std::string scopedDeviceName = m_robotName + "::" + m_controlBoards[n]->key.c_str();
            GazeboYarpPlugins::Handler::getHandler()->removeDevice(scopedDeviceName);
        }

        GazeboYarpPlugins::Handler::getHandler()->removeRobot(m_robotName);
        yarp::os::Network::fini();
    }

    /**
     * Saves the gazebo pointer, creates the device driver
     */
    void GazeboYarpFakeControlBoard::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        yarp::os::Network::init();

        if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
            yError() << "GazeboYarpFakeControlBoard : yarp network does not seem to be available, is the yarpserver running?";
            return;
        }

        if (!_parent) {
            gzerr << "GazeboYarpFakeControlBoard plugin requires a parent.\n";
            return;
        }

        m_robotName = _parent->GetScopedName();
        GazeboYarpPlugins::Handler::getHandler()->setRobot(get_pointer(_parent));

        // Add the gazebo_controlboard device driver to the factory.
        yarp::dev::Drivers::factory().add(new yarp::dev::DriverCreatorOf<yarp::dev::GazeboYarpFakeControlBoardDriver>("gazebo_fakecontrolboard", "controlboardwrapper2", "GazeboYarpFakeControlBoardDriver"));

        //Getting .ini configuration file from sdf
        bool configuration_loaded = false;

        yarp::os::Bottle wrapper_group;
        yarp::os::Bottle driver_group;
        if (_sdf->HasElement("yarpConfigurationFile")) {
            std::string ini_file_name = _sdf->Get<std::string>("yarpConfigurationFile");
            std::string ini_file_path = gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);

            GazeboYarpPlugins::addGazeboEnviromentalVariablesModel(_parent,_sdf,m_pluginParameters);

            bool wipe = false;
            if (ini_file_path != "" && m_pluginParameters.fromConfigFile(ini_file_path.c_str(),wipe))
            {
                m_pluginParameters.put("gazebo_ini_file_path",ini_file_path.c_str());

                wrapper_group = m_pluginParameters.findGroup("WRAPPER");
                if(wrapper_group.isNull()) {
                    yError("GazeboYarpFakeControlBoard : [WRAPPER] group not found in config file\n");
                    return;
                }

                if(m_pluginParameters.check("ROS"))
                {
                    std::string ROS;
                    ROS = std::string ("(") + m_pluginParameters.findGroup("ROS").toString() + std::string (")");
                    wrapper_group.append(yarp::os::Bottle(ROS));
                }

                configuration_loaded = true;
            }

        }

        if (!configuration_loaded) {
            yError() << "GazeboYarpFakeControlBoard : File .ini not found, load failed." ;
            return;
        }

        m_wrapper.open(wrapper_group);

        if (!m_wrapper.isValid()) {
            yError("GazeboYarpFakeControlBoard : wrapper did not open, load failed.");
            m_wrapper.close();
            return;
        }

        if (!m_wrapper.view(m_iWrap)) {
            yError("GazeboYarpFakeControlBoard : wrapper interface not found, load failed.");
            return;
        }

        yarp::os::Bottle *netList = wrapper_group.find("networks").asList();

        if (netList->isNull()) {
            yError("GazeboYarpFakeControlBoard : net list to attach to was not found, load failed.");
            m_wrapper.close();
            return;
        }

        for (int n = 0; n < netList->size(); n++)
        {
            yarp::dev::PolyDriverDescriptor newPoly;

            newPoly.key = netList->get(n).asString();
            std::string scopedDeviceName = m_robotName + "::" + newPoly.key.c_str();
            newPoly.poly = GazeboYarpPlugins::Handler::getHandler()->getDevice(scopedDeviceName);

            if( newPoly.poly != NULL)
            {
                // device already exists, use it, setting it againg to increment the usage counter.
                yWarning("GazeboYarpFakeControlBoard : controlBoard %s already opened.", newPoly.key.c_str());
            }
            else
            {
                driver_group = m_pluginParameters.findGroup(newPoly.key.c_str());
                if (driver_group.isNull()) {
                    yError("GazeboYarpFakeControlBoard : [%s] group not found in config file. Closing wrapper.", newPoly.key.c_str());
                    m_wrapper.close();
                    return;
                }

                m_pluginParameters.put("name", newPoly.key.c_str());
                m_pluginParameters.fromString(driver_group.toString(), false);
                m_pluginParameters.put("robotScopedName", m_robotName);

                if (_sdf->HasElement("initialConfiguration")) {
                    //yDebug()<<"Found initial Configuration: ";
                    std::string configuration_s = _sdf->Get<std::string>("initialConfiguration");
                    m_pluginParameters.put("initialConfiguration", configuration_s.c_str());
                    //yDebug()<<configuration_s;
                }

                 newPoly.poly = new yarp::dev::PolyDriver;
                if(! newPoly.poly->open(m_pluginParameters) || ! newPoly.poly->isValid())
                {
                    yError() << "GazeboYarpFakeControlBoard : controlBoard <" << newPoly.key << "> did not open.";
                    for(int idx=0; idx<m_controlBoards.size(); idx++)
                    {
                        m_controlBoards[idx]->poly->close();
                    }
                    m_wrapper.close();
                    return;
                }
            }
            GazeboYarpPlugins::Handler::getHandler()->setDevice(scopedDeviceName, newPoly.poly);
            m_controlBoards.push(newPoly);
        }

        if (!m_iWrap || !m_iWrap->attachAll(m_controlBoards))
        {
            yError("GazeboYarpFakeControlBoard : error while attaching wrapper to device.");
            m_wrapper.close();
            for (int n = 0; n < netList->size(); n++) {
                std::string scopedDeviceName = m_robotName + "::" + m_controlBoards[n]->key.c_str();
                GazeboYarpPlugins::Handler::getHandler()->removeDevice(scopedDeviceName);
            }
            return;
        }
    }

}
