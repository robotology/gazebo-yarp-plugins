/*Copyright (C) 2007-2015 Istituto Italiano di Tecnologia ADVR & iCub Facility & RBCS Department
 * Authors: Yeshasvi Tirupachuri
 * CopyPolicy:
 */

#include <AccessLinkPose.h>

namespace gazebo
{
AccessLinkPose::AccessLinkPose()
{
    yInfo() << "AccessLinkPose default constructor";
}

AccessLinkPose::~AccessLinkPose()
{
    yInfo() << "AccessLinkPose destructor";
    if(m_network)
        delete m_network;
}

void AccessLinkPose::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    if(m_network != 0)
        return;
    
    m_network = new yarp::os::Network();
    
    if(!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout))
    {
        yError() << "AccessLinkPose::Load error: yarp network is not available, is the yarpserver running?";
        return;
    }
    
    bool configuration_loaded = false;
    
    if(_sdf->HasElement("yarpConfigurationFile"))
    {
        std::string ini_file_name = _sdf->Get<std::string>("yarpConfigurationFile");
        std::string ini_file_path = gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);

        if (ini_file_path != "" && m_parameters.fromConfigFile(ini_file_path.c_str()))
        {
            yInfo() << "Found yarpConfigurationFile: loading from " << ini_file_path ;
            
            //Get number of links from config file
            number_of_links = m_parameters.find("number_of_links").asInt();
            yInfo() << "Number of links : " << number_of_links;
            
            //Get the link names from config file
            link_names_group = m_parameters.findGroup("link_names");
            if(!link_names_group.check("link_names"))
                yError() << "link_names group not found in yarpConfigurationFile";
            else
            {
                yInfo() << "link_names group size : " << link_names_group.size()-1;
                if( (link_names_group.size()-1) != number_of_links)
                    yError() << "Invalid number of parameters in link_names";
                else
                {
                    link_names_vec.resize(number_of_links);
                    for(int i=0; i < number_of_links; i++)
                    {
                        link_names_vec.at(i) = link_names_group.get(i+1).asString();
                        yInfo() << link_names_vec.at(i);
                    }
                }
            }
            
            //Get the link pose type from config file
            link_pose_type_group = m_parameters.findGroup("link_pose_type");
            if(!link_pose_type_group.check("link_pose_type"))
                yError() << "link_pose_type group not found in yarpConfigurationFile";
            else
            {
                yInfo() << "link_pose_type group size : " << link_pose_type_group.size()-1;
                if( (link_pose_type_group.size()-1) != number_of_links)
                    yError() << "Invalid number of parameters in link_pose_type";
                else
                {
                    link_pose_type_vec.resize(number_of_links);
                    for(int i=0; i < number_of_links; i++)
                    {
                        link_pose_type_vec.at(i) = link_pose_type_group.get(i+1).asString();
                        yInfo() << link_pose_type_vec.at(i);
                    }
                }
            }
            configuration_loaded = true;
        }
    }
    
    if(!configuration_loaded)
    {
        yError() << "AccessLinkPose::Load error, could not load yarpConfigurationFile";
        return;
    }

}



}