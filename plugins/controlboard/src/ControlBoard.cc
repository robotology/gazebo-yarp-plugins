/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "ControlBoard.hh"
#include "ControlBoardDriver.h"
#include "ControlBoardLog.h"
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <gazebo/physics/Model.hh>
#include <yarp/dev/IMultipleWrapper.h>

#include <yarp/os/Network.h>

using namespace std;

namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN(GazeboYarpControlBoard)

    GazeboYarpControlBoard::GazeboYarpControlBoard() : m_iWrap(nullptr),
                                                       m_iVirtAnalogSensorWrap(nullptr)
    {}

    GazeboYarpControlBoard::~GazeboYarpControlBoard()
    {
        if (m_iWrap) {
            m_iWrap->detachAll();
            m_iWrap = nullptr;
        }

        if (m_wrapper.isValid()) {
            m_wrapper.close();
        }
        
        if (m_iVirtAnalogSensorWrap) 
        {
            m_iVirtAnalogSensorWrap->detachAll();
            m_iVirtAnalogSensorWrap = nullptr;
        }
        
        if (m_virtAnalogSensorWrapper.isValid())
        {
            m_virtAnalogSensorWrapper.close();
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
            yCError(GAZEBOCONTROLBOARD) << "yarp network does not seem to be available, is the yarpserver running?";
            return;
        }

        if (!_parent) {
            yCError(GAZEBOCONTROLBOARD) << "the plugin requires a parent.";
            return;
        }

        m_robotName = _parent->GetScopedName();
        GazeboYarpPlugins::Handler::getHandler()->setRobot(get_pointer(_parent));

        // Add the gazebo_controlboard device driver to the factory.
        yarp::dev::Drivers::factory().add(new yarp::dev::DriverCreatorOf<yarp::dev::GazeboYarpControlBoardDriver>("gazebo_controlboard", "controlboardwrapper2", "GazeboYarpControlBoardDriver"));

        //Getting .ini configuration file from sdf
        bool configuration_loaded = GazeboYarpPlugins::loadConfigModelPlugin(_parent, _sdf, m_parameters);

        if (!configuration_loaded)
        {
            yCError(GAZEBOCONTROLBOARD) << "File .ini not found, load failed." ;
            return;
        }

        yarp::os::Bottle wrapper_group = m_parameters.findGroup("WRAPPER");
        if(wrapper_group.isNull()) 
        {
            yCDebug(GAZEBOCONTROLBOARD) <<"[WRAPPER] group not found in config file";
        }

        if(m_parameters.check("ROS"))
        {
            std::string ROS;
            ROS = std::string ("(") + m_parameters.findGroup("ROS").toString() + std::string (")");
            wrapper_group.append(yarp::os::Bottle(ROS));
        }

        m_wrapper.open(wrapper_group);

        if (!m_wrapper.isValid()) {
            yCError(GAZEBOCONTROLBOARD) <<"wrapper did not open, load failed.";
            m_wrapper.close();
            return;
        }

        if (!m_wrapper.view(m_iWrap)) {
            yCError(GAZEBOCONTROLBOARD) <<"wrapper interface not found, load failed.";
        }

        yarp::os::Bottle *netList = wrapper_group.find("networks").asList();

        if (netList->isNull()) {
            yCError(GAZEBOCONTROLBOARD) <<"net list to attach to was not found, load failed.";
            m_wrapper.close();
            return;
        }

        yarp::os::Bottle driver_group;
        yarp::os::Bottle virt_group;
        
        m_useVirtAnalogSensor = m_parameters.check("useVirtualAnalogSensor", yarp::os::Value(false)).asBool();
        if (m_useVirtAnalogSensor)
        {
            virt_group = m_parameters.findGroup("VIRTUAL_ANALOG_SERVER");
            if (virt_group.isNull())
            {
                yCError(GAZEBOCONTROLBOARD) <<"[VIRTUAL_ANALOG_SERVER] group not found in config file";
                return;
            }                   

            yarp::os::Bottle& robotName_config = virt_group.addList();
            robotName_config.addString("robotName");
            robotName_config.addString(m_robotName.c_str());
            
            std::string networks = std::string("(") + wrapper_group.findGroup("networks").toString() + std::string(")");
            virt_group.append(yarp::os::Bottle(networks));            
        }

        for (int n = 0; n < netList->size(); n++)
        {
            yarp::dev::PolyDriverDescriptor newPoly;

            newPoly.key = netList->get(n).asString();
            
            // initially deal with virtual analog stuff
            if (m_useVirtAnalogSensor)
            {
                std::string net = std::string("(") + wrapper_group.findGroup(newPoly.key.c_str()).toString() + std::string(")");
                virt_group.append(yarp::os::Bottle(net));                    
            }
            
            std::string scopedDeviceName;
            if(!m_parameters.check("yarpDeviceName"))
            {
                scopedDeviceName = m_robotName + "::" + newPoly.key.c_str();
            }
            else
            {
                scopedDeviceName = m_robotName + "::" + m_parameters.find("yarpDeviceName").asString();
            }
            newPoly.poly = GazeboYarpPlugins::Handler::getHandler()->getDevice(scopedDeviceName);
            if( newPoly.poly != NULL)
            {
                // device already exists, use it, setting it againg to increment the usage counter.
                yCWarning(GAZEBOCONTROLBOARD) << newPoly.key.c_str() << "already opened.";
            }
            else
            {
                driver_group = m_parameters.findGroup(newPoly.key.c_str());
                if (driver_group.isNull()) {
                    yCError(GAZEBOCONTROLBOARD) <<"group not found in config file. Closing wrapper." << newPoly.key.c_str();
                    m_wrapper.close();
                    return;
                }

                m_parameters.put("name", newPoly.key.c_str());
                m_parameters.fromString(driver_group.toString(), false);
                m_parameters.put("robotScopedName", m_robotName);

                if (_sdf->HasElement("initialConfiguration")) {
                    //yCDebug(GAZEBOCONTROLBOARD)<<"Found initial Configuration: ";
                    std::string configuration_s = _sdf->Get<std::string>("initialConfiguration");
                    m_parameters.put("initialConfiguration", configuration_s.c_str());
                    //yCDebug(GAZEBOCONTROLBOARD)<<configuration_s;
                }

                newPoly.poly = new yarp::dev::PolyDriver;
                if(! newPoly.poly->open(m_parameters) || ! newPoly.poly->isValid())
                {
                    yCError(GAZEBOCONTROLBOARD) << newPoly.key << "> did not open.";
                    for(int idx=0; idx<m_controlBoards.size(); idx++)
                    {
                        m_controlBoards[idx]->poly->close();
                    }
                    m_wrapper.close();
                    return;
                }
            }

            //Register the device with the given name
            if(!GazeboYarpPlugins::Handler::getHandler()->setDevice(scopedDeviceName, newPoly.poly))
            {
                yCError(GAZEBOCONTROLBOARD) << "failed setting scopedDeviceName(=" << scopedDeviceName << ")";
                return;
            }
            yCInfo(GAZEBOCONTROLBOARD) << "Registered YARP device with instance name:" << scopedDeviceName;

            m_controlBoards.push(newPoly);
        }
        
        if (m_useVirtAnalogSensor)
        {
            m_virtAnalogSensorWrapper.open(virt_group);

            if (!m_virtAnalogSensorWrapper.isValid()) 
            {
                yCError(GAZEBOCONTROLBOARD) << "Virtual analog sensor wrapper did not open, load failed.";
                m_virtAnalogSensorWrapper.close();
                return;
            }

            if (!m_virtAnalogSensorWrapper.view(m_iVirtAnalogSensorWrap)) 
            {
                yCError(GAZEBOCONTROLBOARD) << "Could not view the IVirtualAnalogSensor interface";
                return;
            }
            
            if (!m_iVirtAnalogSensorWrap->attachAll(m_controlBoards))
            {
                yCError(GAZEBOCONTROLBOARD) << "Could not attach VirtualAnalogSensor interface to controlboards";
                return;
            }
        }           
        
        if (!m_iWrap || !m_iWrap->attachAll(m_controlBoards))
        {
            yCError(GAZEBOCONTROLBOARD) << "error while attaching wrapper to device.";
            m_wrapper.close();
            if (m_useVirtAnalogSensor)
            {
                m_virtAnalogSensorWrapper.close();
            }
            for (int n = 0; n < netList->size(); n++) {
                std::string scopedDeviceName = m_robotName + "::" + m_controlBoards[n]->key.c_str();
                GazeboYarpPlugins::Handler::getHandler()->removeDevice(scopedDeviceName);
            }
            return;
        }
    }
} // namespace gazebo
