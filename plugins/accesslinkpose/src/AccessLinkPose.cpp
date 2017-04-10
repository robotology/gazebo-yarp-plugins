/*Copyright (C) 2007-2015 Istituto Italiano di Tecnologia ADVR & iCub Facility & RBCS Department
 * Authors: Yeshasvi Tirupachuri
 * CopyPolicy:
 */

#include <AccessLinkPose.h>

namespace gazebo
{
AccessLinkPose::AccessLinkPose()
{
    //yInfo() << "AccessLinkPose default constructor";
}

AccessLinkPose::~AccessLinkPose()
{
    //yInfo() << "AccessLinkPose destructor";
    if(m_network)
        delete m_network;
}

void AccessLinkPose::getLinkPoses()
{
    //yInfo() << "Getting link new pose values";
    link_poses.clear();
    
    for(int i=0; i < links.size(); i++)
    {
        link = links.at(i);
        
        if(link_pose_type_vec.at(i) == "cog" || link_pose_type_vec.at(i) == "CoG" || link_pose_type_vec.at(i) == "WorldCoG")
        {
            link_pose = link->GetWorldCoGPose();
            link_poses.push_back(link_pose);
        }
        else if(link_pose_type_vec.at(i) == "inertial" || link_pose_type_vec.at(i) == "Inertial" || link_pose_type_vec.at(i) == "WorldInertial")
        {
            link_pose = link->GetWorldInertialPose();
            link_poses.push_back(link_pose);
        }
        else if(link_pose_type_vec.at(i) == "world" || link_pose_type_vec.at(i) == "World")
        {
            link_pose = link->GetWorldPose();
            link_poses.push_back(link_pose);
        }
    }
    
    //yInfo() << "Link poses size : " << link_poses.size();
    yarp::os::Bottle& pose_bottle = pose_output_port.prepare();
    pose_bottle.clear();
    
    for(int p=0; p < links.size(); p++)
    {
        //std::cout << links.at(p)->GetName() << " pose " << link_poses.at(p) << std::endl;
        
        std::string link_name = links.at(p)->GetName();
        pose_bottle.addString(link_name);
        
        gazebo::math::Pose pose = link_poses.at(p);
        gazebo::math::Vector3 position = pose.pos;
        gazebo::math::Quaternion orientation = pose.rot;
        //std::cout << double(orientation.x);
        
        pose_bottle.addDouble(position[0]);
        pose_bottle.addDouble(position[1]);
        pose_bottle.addDouble(position[2]);
        
        pose_bottle.addDouble(orientation.x);
        pose_bottle.addDouble(orientation.y);
        pose_bottle.addDouble(orientation.z);
        pose_bottle.addDouble(orientation.w);
    }
    
    //yInfo() << "Pose Bottle : " << pose_bottle.toString().c_str();
    pose_output_port.writeStrict();    
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
            
            std::string port_name = m_parameters.find("port_name").asString();
            pose_output_port.open(port_name);
            
            //Get number of links from config file
            number_of_links = m_parameters.find("number_of_links").asInt();
            //yInfo() << "Number of links : " << number_of_links;
            
            //Get the link names from config file
            link_names_group = m_parameters.findGroup("link_names");
            if(!link_names_group.check("link_names"))
                yError() << "link_names group not found in yarpConfigurationFile";
            else
            {
                //yInfo() << "link_names group size : " << link_names_group.size()-1;
                if( (link_names_group.size()-1) != number_of_links)
                    yError() << "Invalid number of parameters in link_names";
                else
                {
                    link_names_vec.resize(number_of_links);
                    for(int i=0; i < number_of_links; i++)
                    {
                        link_names_vec.at(i) = link_names_group.get(i+1).asString();
                        //yInfo() << link_names_vec.at(i);
                    }
                }
            }
            
            //Get the link pose type from config file
            link_pose_type_group = m_parameters.findGroup("link_pose_type");
            if(!link_pose_type_group.check("link_pose_type"))
                yError() << "link_pose_type group not found in yarpConfigurationFile";
            else
            {
                //yInfo() << "link_pose_type group size : " << link_pose_type_group.size()-1;
                if( (link_pose_type_group.size()-1) != number_of_links)
                    yError() << "Invalid number of parameters in link_pose_type";
                else
                {
                    link_pose_type_vec.resize(number_of_links);
                    for(int i=0; i < number_of_links; i++)
                    {
                        link_pose_type_vec.at(i) = link_pose_type_group.get(i+1).asString();
                        //yInfo() << link_pose_type_vec.at(i);
                    }
                }
            }
            configuration_loaded = true;
        }
        
        model = _model;
        all_links = model->GetLinks();
        //yInfo() << "Total number of links in the model : " << all_links.size();
        
        //Get links for which pose is needed            
        for(int l = 0; l < number_of_links; l++)
        {
            for(int i=0; i < all_links.size(); i++)
            {
                std::string candidate_link_name = all_links[i]->GetScopedName();
                //yInfo() << "Candidate link name : " << candidate_link_name;
                
                std::size_t lastcolon = candidate_link_name.rfind(":");
                std::string unscoped_link_name = candidate_link_name.substr(lastcolon+1,std::string::npos);
                
                if(unscoped_link_name == link_names_vec.at(l))
                {
                    link = all_links[i];
                    //yInfo() << "Found link : " << link->GetName();
                    links.push_back(link);
                    break;
                }
            }
            if(links.size() != l+1)
            {
                yError() << link_names_vec.at(l) << " not found in the gazebo model";\
                return;
            }
        }
        
        //yInfo() << "Pose links size : " << links.size();   
    }
    
    if(!configuration_loaded)
    {
        yError() << "AccessLinkPose::Load error, could not load yarpConfigurationFile";
        return;
    }
    
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&AccessLinkPose::getLinkPoses, this));

}



}