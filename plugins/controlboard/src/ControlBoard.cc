/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "ControlBoard.hh"
#include "ControlBoardDriver.h"
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <gazebo/physics/Model.hh>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/IRobotDescription.h>
#include <yarp/os/Property.h>

using namespace std;
namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN(GazeboYarpControlBoard)

    GazeboYarpControlBoard::GazeboYarpControlBoard() : m_iWrap(0)
    {}

    GazeboYarpControlBoard::~GazeboYarpControlBoard()
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
    void GazeboYarpControlBoard::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        yarp::os::Network::init();

        if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
            yError() << "GazeboYarpControlBoard : yarp network does not seem to be available, is the yarpserver running?";
            return;
        }

        if (!_parent) {
            gzerr << "GazeboYarpControlBoard plugin requires a parent.\n";
            return;
        }

        m_robotName = _parent->GetScopedName();
        GazeboYarpPlugins::Handler::getHandler()->setRobot(get_pointer(_parent));

        // Add the gazebo_controlboard device driver to the factory.
        yarp::dev::Drivers::factory().add(new yarp::dev::DriverCreatorOf<yarp::dev::GazeboYarpControlBoardDriver>("gazebo_controlboard", "controlboardwrapper2", "GazeboYarpControlBoardDriver"));

        //Getting .ini configuration file from sdf
        bool configuration_loaded = false;

        yarp::os::Bottle wrapper_group;
        yarp::os::Bottle driver_group;
        if (_sdf->HasElement("yarpConfigurationFile")) {
            std::string ini_file_name = _sdf->Get<std::string>("yarpConfigurationFile");
            std::string ini_file_path = gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);

            GazeboYarpPlugins::addGazeboEnviromentalVariablesModel(_parent,_sdf,m_parameters);

            bool wipe = false;
            if (ini_file_path != "" && m_parameters.fromConfigFile(ini_file_path.c_str(),wipe))
            {
                m_parameters.put("gazebo_ini_file_path",ini_file_path.c_str());

                wrapper_group = m_parameters.findGroup("WRAPPER");
                if(wrapper_group.isNull()) {
                    yError("GazeboYarpControlBoard : [WRAPPER] group not found in config file\n");
                    return;
                }

                if(m_parameters.check("ROS"))
                {
                    yarp::os::ConstString ROS;
                    ROS = yarp::os::ConstString ("(") + m_parameters.findGroup("ROS").toString() + yarp::os::ConstString (")");
                    wrapper_group.append(yarp::os::Bottle(ROS));
                }

                configuration_loaded = true;
            }

        }

        if (!configuration_loaded) {
            yError() << "GazeboYarpControlBoard : File .ini not found, load failed." ;
            return;
        }

        m_wrapper.open(wrapper_group);

        if (!m_wrapper.isValid()) {
            yError("GazeboYarpControlBoard : wrapper did not open, load failed.");
            m_wrapper.close();
            return;
        }

        if (!m_wrapper.view(m_iWrap)) {
            yError("GazeboYarpControlBoard : wrapper interface not found, load failed.");
            return;
        }

        yarp::os::Bottle *netList = wrapper_group.find("networks").asList();

        if (netList->isNull()) {
            yError("GazeboYarpControlBoard : net list to attach to was not found, load failed.");
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
                yWarning("GazeboYarpControlBoard : controlBoard %s already opened.", newPoly.key.c_str());
            }
            else
            {
                driver_group = m_parameters.findGroup(newPoly.key.c_str());
                if (driver_group.isNull()) {
                    yError("GazeboYarpControlBoard : [%s] group not found in config file. Closing wrapper.", newPoly.key.c_str());
                    m_wrapper.close();
                    return;
                }

                m_parameters.put("name", newPoly.key.c_str());
                m_parameters.fromString(driver_group.toString(), false);
                m_parameters.put("robotScopedName", m_robotName);

                if (_sdf->HasElement("initialConfiguration")) {
                    //yDebug()<<"Found initial Configuration: ";
                    std::string configuration_s = _sdf->Get<std::string>("initialConfiguration");
                    m_parameters.put("initialConfiguration", configuration_s.c_str());
                    //yDebug()<<configuration_s;
                }

                 newPoly.poly = new yarp::dev::PolyDriver;
                if(! newPoly.poly->open(m_parameters) || ! newPoly.poly->isValid())
                {
                    yError() << "GazeboYarpControlBoard : controlBoard <" << newPoly.key << "> did not open.";
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
            yError("GazeboYarpControlBoard : error while attaching wrapper to device.");
            m_wrapper.close();
            for (int n = 0; n < netList->size(); n++) {
                std::string scopedDeviceName = m_robotName + "::" + m_controlBoards[n]->key.c_str();
                GazeboYarpPlugins::Handler::getHandler()->removeDevice(scopedDeviceName);
            }
            return;
        }
        
        if (registerRobotDescription(wrapper_group.find("name").toString()) == false)
        {
            yError() << "GazeboYarpControlBoard : failed to register the device to a RobotDescriptionServer";
            //we may want to stop the execution?
            //m_wrapper.close();
            return;
        }
    }

    bool GazeboYarpControlBoard::registerRobotDescription(std::string wrapper_port_name)
    {
        //here starts the robot description stuff...
        yarp::dev::PolyDriver* dd_descClnt=0;
        yarp::dev::PolyDriver* dd_descSrv=0;
        yarp::dev::IRobotDescription* idesc=0;
        
        //first we try to open the robotDescriptionClient, because maybe an external server is already running
        dd_descClnt = new yarp::dev::PolyDriver;
        yarp::os::Property clnt_opt;
        clnt_opt.put("device", "robotDescriptionClient");
        clnt_opt.put("local", std::string(wrapper_port_name+"/robotDescriptionClient"));
        clnt_opt.put("remote", "/robotDescription");

        bool b_client = false;
        if (dd_descClnt->open(clnt_opt) && dd_descClnt->isValid())
        {
            yInfo() << "External robotDescriptionServer was found. Connection succesful.";
            b_client = true;
            dd_descClnt->view(idesc);
        }
        else
        {
            yInfo() << "External robotDescriptionServer was not found. Opening a new RobotDescriptionServer.";
        }

        //if the previous operation failed, then retry opening first a robotDescriptionServer and then a robotDescriptionClient
        if (b_client == false)
        {
            dd_descSrv = new yarp::dev::PolyDriver;
            yarp::os::Property srv_opt;
            srv_opt.put("device", "robotDescriptionServer");
            srv_opt.put("local", "/robotDescription");

            bool b_server = false;
            if (dd_descSrv->open(srv_opt) && dd_descSrv->isValid())
            {
                b_server = true;
            }
            else
            {
                //unable to open a robotDescriptionServer, this is a critical failure!
                delete dd_descSrv;
                dd_descSrv = 0;
                yError() << "Unable to open robotDescriptionServer";
                return false;
            }

            //robotDescriptionServer is running, so let's retry to open the robotDescriptionClient
            if (dd_descClnt->open(clnt_opt) && dd_descClnt->isValid())
            {
                b_client = true;
                dd_descClnt->view(idesc);
            }
            else
            {
                //unable to open the robotDescriptionClient, this is a critical failure!
                delete dd_descClnt;
                dd_descClnt = 0;
                yError() << "Unable to open robotDescriptionClient";
                return false;
            }
        }
        
        //we can now register the device
        yarp::dev::DeviceDescription dev_desc;
        dev_desc.device_name=wrapper_port_name;
        dev_desc.device_type="controlboardwrapper2";
        idesc->registerDevice(dev_desc);
        
        //and finally close the robotDescriptionClient
        dd_descClnt->close();
        delete dd_descClnt;
        dd_descClnt = 0;
        return true;
    }
}

