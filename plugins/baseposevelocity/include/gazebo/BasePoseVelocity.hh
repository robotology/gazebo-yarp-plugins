/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBO_YARP_BASEPOSEVELOCITY_HH
#define GAZEBO_YARP_BASEPOSEVELOCITY_HH

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
    class GazeboYarpBasePoseVelocity : public ModelPlugin
    {
    public:
        GazeboYarpBasePoseVelocity();
        virtual ~GazeboYarpBasePoseVelocity();
        
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        
    private:
        yarp::dev::PolyDriver m_deviceDriver;  // devdriver
        yarp::dev::PolyDriver m_networkDevice; //devwrapper
        yarp::dev::IMultipleWrapper* m_networkWrapper; //imultwrapper
        
        std::string m_robot;
        
        yarp::os::Property m_config;
               
    };
}

#endif
