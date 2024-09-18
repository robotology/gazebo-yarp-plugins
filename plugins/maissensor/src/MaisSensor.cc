/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "MaisSensor.hh"
#include "MaisSensorDriver.h"
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <gazebo/physics/Model.hh>
#include <yarp/dev/IMultipleWrapper.h>

#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace std;
namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN(GazeboYarpMaisSensor)

GazeboYarpMaisSensor::GazeboYarpMaisSensor() : m_iWrap(0)
{

}

GazeboYarpMaisSensor::~GazeboYarpMaisSensor()
{
    if (m_iWrap)
    {
        m_iWrap->detachAll();
        m_iWrap = 0;
    }
    if (m_wrapper.isValid())
        m_wrapper.close();

    if (m_deviceRegistered)
    {
        if (m_parameters.check("disableImplicitNetworkWrapper"))
        {
            GazeboYarpPlugins::Handler::getHandler()->removeDevice(m_scopedDeviceName);
        }
        else
        {
            GazeboYarpPlugins::Handler::getHandler()->removeDevice(m_sensorName);
        }
    }
    GazeboYarpPlugins::Handler::getHandler()->removeRobot(m_robotName);

    yarp::os::Network::fini();
}


void GazeboYarpMaisSensor::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    yarp::os::Network::init();

    if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
        yError() << "GazeboYarpMaisSensor::Load error: yarp network does not seem to be available, is the yarpserver running?";
        return;
    }

    if (!_parent) {
        gzerr << "GazeboYarpMaisSensor plugin requires a parent.\n";
        return;
    }

    m_robotName = _parent->GetScopedName();
    GazeboYarpPlugins::Handler::getHandler()->setRobot(get_pointer(_parent));

    // Add the gazebo_controlboard device driver to the factory.
    yarp::dev::Drivers::factory().add(new yarp::dev::DriverCreatorOf<yarp::dev::GazeboYarpMaisSensorDriver>("gazebo_maissensor", "analogServer", "GazeboYarpMaisSensorDriver"));

    // Getting .ini configuration file parameters from sdf
    bool configuration_loaded = GazeboYarpPlugins::loadConfigModelPlugin(_parent, _sdf, m_parameters);

    if (!configuration_loaded)
    {
        yError() << "GazeboYarpMaisSensor : File .ini not found, load failed." ;
        return;
    }

    bool disable_wrapper = m_parameters.check("disableImplicitNetworkWrapper");

    if (disable_wrapper && !m_parameters.check("yarpDeviceName"))
    {
        yError() << "GazeboYarpMaisSensor : missing yarpDeviceName parameter for one device in robot " << m_robotName;
        return;
    }

    if (!disable_wrapper)
    {
        yarp::os::Bottle wrapper_group = m_parameters.findGroup("WRAPPER");
        if(wrapper_group.isNull())
        {
            yError("GazeboYarpMaisSensor : [WRAPPER] group not found in config file\n");
            return;
        }

        if(m_parameters.check("ROS"))
        {
            std::string ROS;
            ROS = std::string ("(") + m_parameters.findGroup("ROS").toString() + std::string (")");
            wrapper_group.append(yarp::os::Bottle(ROS));
        }

        //Open the wrapper
        if( m_wrapper.open(wrapper_group) )
        {
        }
        else
        {
            yError()<<"GazeboYarpMaisSensor Plugin failed: error in opening yarp driver wrapper";
            return;
        }

        yarp::os::Bottle *netList = wrapper_group.find("networks").asList();

        if (netList->isNull())
        {
            yError("GazeboYarpMaisSensor : net list to attach to was not found, load failed.");
            m_wrapper.close();
            return;
        }

        //---------------------------------------------
        yarp::dev::PolyDriverDescriptor newPoly;

        if (netList->size()!=1)
        {
            yError("GazeboYarpMaisSensor: size of 'networks' parameter cannot be != 1");
        }

        newPoly.key = netList->get(0).asString();
        m_sensorName = m_robotName + "::" + newPoly.key.c_str();
        newPoly.poly = GazeboYarpPlugins::Handler::getHandler()->getDevice(m_sensorName);

        yarp::os::Bottle driver_group;
        if( newPoly.poly != NULL)
        {
            // device already exists, use it, setting it againg to increment the usage counter.
            yError("mais %s already opened", newPoly.key.c_str());
        }
        else
        {
            driver_group = m_parameters.findGroup(newPoly.key.c_str());
            if (driver_group.isNull())
            {
                yError("GazeboYarpMaisSensor::Load  Error: [%s] group not found in config file. Closing wrapper for device [%s].", newPoly.key.c_str(), m_sensorName.c_str());
                return;
            }

            m_parameters.put("name", newPoly.key.c_str());
            m_parameters.fromString(driver_group.toString(), false);
            m_parameters.put("robotScopedName", m_robotName);
            m_parameters.put("device","gazebo_maissensor");

            newPoly.poly = new yarp::dev::PolyDriver;
            if(! newPoly.poly->open(m_parameters) || ! newPoly.poly->isValid())
            {
                yError() << "mais <" << newPoly.key << "> did not open!!";
                newPoly.poly->close();
                return;
            }
        }
        GazeboYarpPlugins::Handler::getHandler()->setDevice(m_sensorName, newPoly.poly);

        //------------------
        if (!m_wrapper.isValid())
        {
            yError("GazeboYarpMaisSensor: wrapper did not open");
        }

        if (!m_wrapper.view(m_iWrap))
        {
            yError("Wrapper interface not found");
        }

        //Attach the driver to the wrapper
        yarp::dev::PolyDriverList driver_list;

        m_deviceRegistered = true;
        yInfo() << "GazeboYarpMaisSensor: Registered YARP device with instance name:" << m_sensorName;
        driver_list.push(newPoly.poly,"dummy");

        if( m_iWrap->attachAll(driver_list) ) {
        } else {
            yError() << "GazeboYarpMaisSensor : error in connecting wrapper and device ";
        }
    }
    else
    {
        m_yarpDeviceName = m_parameters.find("yarpDeviceName").asString();
        m_scopedDeviceName = m_robotName + "::" + m_yarpDeviceName;

        m_parameters.put("name", m_scopedDeviceName);
        m_parameters.put("robotScopedName", m_robotName);
        m_parameters.put("device","gazebo_maissensor");

        if (!m_maisEncodersDriver.open(m_parameters) || ! m_maisEncodersDriver.isValid())
        {
            yError() << "mais <" << m_yarpDeviceName.c_str() << "> did not open.";
            m_maisEncodersDriver.close();
            return;
        }

        //Register the device with the given name
        if(!GazeboYarpPlugins::Handler::getHandler()->setDevice(m_scopedDeviceName, &m_maisEncodersDriver))
        {
            yError() << "GazeboYarpMaisSensor: failed setting scopedDeviceName(=" << m_scopedDeviceName << ")";
            return;
        }
        m_deviceRegistered = true;
        yInfo() << "GazeboYarpMaisSensor: Registered YARP device with instance name:" << m_scopedDeviceName;

    }
}

}
