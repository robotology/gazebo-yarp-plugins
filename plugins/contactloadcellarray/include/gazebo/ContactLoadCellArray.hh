/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_CONTACTLOADCELLARRAY_HH
#define GAZEBOYARP_CONTACTLOADCELLARRAY_HH

#include <gazebo/common/Plugin.hh>
#include <string>
#include <yarp/dev/PolyDriverList.h>

#include <sdf/sdf_config.h>
#include <sdf/Element.hh>
#include <sdf/Param.hh>


namespace yarp 
{
    namespace dev
    {
        class IMultipleWrapper;
    }
}

namespace gazebo
{
    class GazeboYarpContactLoadCellArray : public ModelPlugin
    {
        public:
        GazeboYarpContactLoadCellArray();
        virtual ~GazeboYarpContactLoadCellArray();
    
       /**
        * Saves the gazebo pointer, creates the device driver
        */
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    
        private:
       
       /**
        * Polydriver instance to connect to analog sensor server
        */
        yarp::dev::PolyDriver m_devWrapper;
        
       /**
        * Wrapper to attach all necessary devices
        */
        yarp::dev::IMultipleWrapper *m_imultwrapper;
        
       /**
        * Polydriver instance to instantiate the device driver
        */
        yarp::dev::PolyDriver m_devDriver;
    
       /**
        * YARP property to store information from config file
        */
        yarp::os::Property m_parameters;    
        
       /**
        * Network interface to connect to
        */
        std::string m_wrapperName;
        
       /**
        * Device driver name to be instantiated
        */
        std::string m_sensorDriverName;
 
       /**
        * Name of the robot model loaded by model plugin 
        */
        std::string m_gazeboRobotName;
   
    };
}


#endif 
