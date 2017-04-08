/*Copyright (C) 2007-2015 Istituto Italiano di Tecnologia ADVR & iCub Facility & RBCS Department
 * Authors: Yeshasvi Tirupachuri
 * CopyPolicy:
 */

#ifndef YARPGAZEBO_ACCESSLINKPOSE_H
#define YARPGAZEBO_ACCESSLINKPOSE_H

#include <gazebo/gazebo.hh>
#include <GazeboYarpPlugins/common.h>

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

namespace gazebo
{
    class AccessLinkPose : public ModelPlugin
    {
    private: 
        
        gazebo::event::ConnectionPtr updateConnection;
        
        yarp::os::Network *m_network;
        yarp::os::Property m_parameters;
        
        int number_of_links;
        std::vector<std::string> link_names_vec;
        std::vector<std::string> link_pose_type_vec;
        yarp::os::Bottle link_names_group;
        yarp::os::Bottle link_pose_type_group;
               
    public:
        AccessLinkPose(); 
        ~AccessLinkPose();
        void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);
        void onUpdate(const gazebo::common::UpdateInfo& /*_info*/);
    };
    
    GZ_REGISTER_MODEL_PLUGIN(AccessLinkPose);
}

#endif