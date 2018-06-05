/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBO_YARP_BASESTATE_HH
#define GAZEBO_YARP_BASESTATE_HH

#include <string>

#include <gazebo/common/Plugin.hh>
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <yarp/dev/Wrapper.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>

namespace yarp
{
    namespace dev
    {
        class IMultipleWrapper;
    }
}

namespace gazebo
{
    class GazeboYarpBaseState : public ModelPlugin
    {
    public:
        GazeboYarpBaseState();
        virtual ~GazeboYarpBaseState();
        
        /**
         * Loads robot model, reads configuration, 
         * opens network wrapper device and opens device driver
         */
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        
    private:
        yarp::dev::PolyDriver m_deviceDriver;           ///< Device driver for getting base link state
        yarp::dev::PolyDriver m_networkDevice;          ///< Network wrapper device to send state through YARP server
        yarp::dev::IMultipleWrapper* m_networkWrapper;  ///< Interface to attach device driver to network device
        std::string m_robot;                            ///< name of robot model
        yarp::os::Property m_config;                    ///< Property to read configuration
               
    };
}

#endif

