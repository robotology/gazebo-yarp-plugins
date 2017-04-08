/*Copyright (C) 2007-2015 Istituto Italiano di Tecnologia ADVR & iCub Facility & RBCS Department
 * Authors: Yeshasvi Tirupachuri
 * CopyPolicy:
 */

#ifndef YARPGAZEBO_ACCESSLINKPOSE_H
#define YARPGAZEBO_ACCESSLINKPOSE_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/math/gzmath.hh>
#include <gazebo/math/Vector3.hh>
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
        
        gazebo::physics::ModelPtr model;
        gazebo::physics::Link_V all_links;
        
        gazebo::physics::LinkPtr link;
        gazebo::physics::Link_V links;
        
        gazebo::math::Pose link_pose;
        std::vector<gazebo::math::Pose> link_poses;
               
    public:
        AccessLinkPose(); 
        ~AccessLinkPose();
        void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);
        void getLinkPoses();
    };
    
    GZ_REGISTER_MODEL_PLUGIN(AccessLinkPose);
}

#endif