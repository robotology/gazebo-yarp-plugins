/*
 * Copyright (C) 2007-2015 Istituto Italiano di Tecnologia RBCS, ADVR & iCub Facility
 * Authors: Mirko Ferrati, Cheng Fang, Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */



#include "Objects.hh"
#include "ObjectsServerImpl.h"
#include "common.h"

#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/math/Vector3.hh>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/Network.h>

#include <sstream>

GZ_REGISTER_MODEL_PLUGIN(gazebo::GazeboYarpObjects)

namespace gazebo {

GazeboYarpObjects::GazeboYarpObjects() : ModelPlugin(), m_network(),node(new transport::Node)
{
}

GazeboYarpObjects::~GazeboYarpObjects()
{
}

void GazeboYarpObjects::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    gazeboYarpObjectsLoad(_parent,_sdf);
}

void GazeboYarpObjects::gazeboYarpObjectsLoad(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    m_model = _parent;
    m_world = _parent->GetWorld();

    m_portName = "/world";

    m_rpcPort = new yarp::os::Port();
    if (!m_rpcPort) {
        std::cerr << "GazeboYarpObjects: Failed to create rpc port." << std::endl;
        cleanup();
        return;
    }

    m_clockServer = new ObjectsServerImpl(*this);
    if (!m_clockServer) {
        std::cerr << "GazeboYarpObjects: Could not create Objects Server." << std::endl;
        cleanup();
        return;
    }

    if (!m_clockServer->yarp().attachAsServer(*m_rpcPort)) {
        std::cerr << "GazeboYarpObjects: Failed to attach Objects Server to RPC port." << std::endl;
        cleanup();
        return;
    }

    if (!m_rpcPort->open(m_portName)) {
        std::cerr << "GazeboYarpObjects: Failed to open rpc port " << (m_portName) << std::endl;
        cleanup();
        return;
    }
    this->m_sdf=_sdf;
    node->Init();
    createHandle();
}


bool gazebo::GazeboYarpObjects::createHandle()
{
    
    sdf::SDF handleSDF;
    std::stringstream ss;
    ss << "<sdf version ='1.4'>\
    <model name ='object_handle'>\
        <pose>0 0 0 0 0 0</pose>\
        <link name ='object_handle_link'>\
            <pose>0 0 0 0 0 0</pose>\
            <collision name='collision'>\
                <geometry>\
                    <box>\
                        <size>0.00001 0.00001 0.00001</size>\
                    </box>\
                </geometry>\
            </collision>\
            <inertial>\
                <mass>0.00001</mass>\
                <inertia>\
                    <ixx>0.00001</ixx>\
                    <ixy>0</ixy>\
                    <ixz>0</ixz>\
                    <iyy>0.00001</iyy>\
                    <iyz>0</iyz>\
                    <izz>0.00001</izz>\
                </inertia>\
            </inertial>\
        </link>\
    </model>\
    </sdf>";
    std::string handleSDF_str = ss.str();
    handleSDF.SetFromString(handleSDF_str);
    m_world->InsertModelSDF(handleSDF);
    
    return true;
}

void GazeboYarpObjects::cleanup()
{
    if (m_rpcPort) {
        m_rpcPort->close();
        delete m_rpcPort; m_rpcPort = 0;
    }

    if (m_clockServer) {
        delete m_clockServer; m_clockServer = 0;
    }
}

bool hasEnding (std::string const &fullString, std::string const &ending)
{
//     std::cout<<fullString<<" "<<ending<<std::endl;
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

gazebo::physics::LinkPtr getLinkInModel(gazebo::physics::ModelPtr model, std::string link_name)
{
    gazebo::physics::Link_V model_links = model->GetLinks();
    for(int i=0; i < model_links.size(); i++ ) {
        std::string candidate_link = model_links[i]->GetScopedName();
        
//         std::cout<<candidate_link<<std::endl;
        if( hasEnding(candidate_link,"::"+link_name) ) {
            return model_links[i];
        }
    }

    return gazebo::physics::LinkPtr();
}

gazebo::physics::LinkPtr getClosestLinkInModel(gazebo::physics::ModelPtr model, gazebo::math::Pose pose)
{
    int index=-1;
    double min_norm=100000;
    gazebo::physics::Link_V model_links = model->GetLinks();
    for(int i=0; i < model_links.size(); i++ ) 
    {
        std::string candidate_link = model_links[i]->GetScopedName();
        double norm = pose.pos.Distance(model_links[i]->GetWorldPose().pos);
        std::cout<<candidate_link<<" "<<norm<<std::endl;
        if (/*norm<0.25 && */norm<min_norm)
        {
            min_norm = norm;
            index = i;
            //         std::string candidate_link = model_links[i]->GetScopedName();
            //         std::cout<<candidate_link<<std::endl;
        }
    }
    if (index>-1)
        return model_links[index];
    else
        return gazebo::physics::LinkPtr();
}

bool gazebo::GazeboYarpObjects::attach(const std::string& link_name, const std::string& object_name)
{
    if (joints_attached.count(object_name+"_attached_joint"))
    {
        std::cout<<"objects is already attached!!"<<std::endl;
        return false;
    }
    physics::ModelPtr object_model = m_world->GetModel(object_name);

    if( !object_model )
    {
        std::cout<<"could not get the model for "<<object_name<<"!!"<<std::endl;
        return false;
    }

    gazebo::physics::LinkPtr parent_link = getLinkInModel(m_model,link_name);

    if (!parent_link)
    {
        std::cout<<"could not get the links for "<<parent_link<<std::endl;
        return false;
    }
    link_object_map[link_name]=object_name;
    
    gazebo::physics::Collision_V collisions=  parent_link->GetCollisions();
    for (int j=0;j<collisions.size();j++)
    {
        collisions_str.push_back(collisions[j]->GetScopedName());
        std::cout<<collisions[j]->GetScopedName()<<std::endl;
    }

    //     collisions_str.push_back("bigman::LWrMot3::LWrMot3_collision_LWrMot3_1");
    physics::ContactManager *mgr =
    m_world->GetPhysicsEngine()->GetContactManager();
    std::string topic = mgr->CreateFilter("objects_topic", collisions_str);
    std::cout<<topic<<std::endl;
//     if (!this->contactSub)
//     {
        contactSub = node->Subscribe(topic, &GazeboYarpObjects::OnContacts, this);
//     }
    
    return true;
}

bool GazeboYarpObjects::attach_impl(std::string link_name,std::string object_name, math::Pose handle_pose, math::Vector3 normal)
{
    physics::LinkPtr parent_link = getLinkInModel(m_model,link_name);
    
    if( !parent_link ) {
        std::cout<<"could not get the links for "<<parent_link<<std::endl;
        return false;
    }
    math::Pose parent_link_pose = parent_link->GetWorldPose();
    physics::ModelPtr object_model = m_world->GetModel(object_name);
    
    if( !object_model )
    {
        std::cout<<"could not get the model for "<<object_name<<"!!"<<std::endl;
        return false;
    }
    
    //TODO disable collisions on the hand, not on the object!!
    gazebo::physics::Link_V model_links = object_model->GetLinks();
    for(int i=0; i < model_links.size(); i++ ) 
        model_links[i]->SetCollideMode("none");

//     math::Pose handle_pose = Get_handle_pose(object_model, parent_link->GetWorldCoGPose(), link_name, width, height, length);
    
//     if( handle_pose.pos.x == 0.0 && handle_pose.pos.y == 0.0 && handle_pose.pos.z == 0.0 && handle_pose.rot.GetRoll() == 0.0 && handle_pose.rot.GetPitch() == 0.0 && handle_pose.rot.GetYaw() == 0.0 ) {
//         std::cout<<"The hand posture is NOT suitable to grasp the specified object" <<std::endl;
//         return false;
//     }
    
    physics::ModelPtr object_handle_model = m_world->GetModel("object_handle");
    physics::LinkPtr object_handle_link = getLinkInModel(object_handle_model,"object_handle_link");
    
    object_handle_link->SetWorldPose(handle_pose);
    
    physics::JointPtr object_joint;
    object_joint = m_world->GetPhysicsEngine()->CreateJoint("revolute", object_model);
    if( !object_joint ) {
        std::cout<<"could not create joint for the object!!"<<std::endl;
        return false;
    }
    
    physics::LinkPtr object_link = getLinkInModel(object_model,"link");
    
    object_joint->SetName(object_name+"_attached_handle_joint");
    joints_attached[object_name+"_attached_handle_joint"]=object_joint;
    object_joint->Load(object_link, object_handle_link, math::Pose());
    object_joint->Attach(object_link, object_handle_link);
    object_joint->SetHighStop(0, 0);
    object_joint->SetLowStop(0, 0);
    
    
    physics::JointPtr joint;
    joint = m_world->GetPhysicsEngine()->CreateJoint("revolute", m_model);
    if( !joint ) {
        std::cout<<"could not create joint!!"<<std::endl;
        return false;
    }
    joint->SetName(object_name+"_attached_joint");
    joints_attached[object_name+"_attached_joint"]=joint;
    math::Pose offset(0,0,-0.15,0,0,0);
    math::Pose offset_pose=parent_link_pose*offset;
    
//     gazebo::physics::LinkPtr object_link = getClosestLinkInModel(object_model,parent_link_pose);
    object_handle_link->SetWorldPose(offset_pose);
    joint->Load(parent_link, object_handle_link, math::Pose());
    joint->Attach(parent_link, object_handle_link);
    joint->SetHighStop(0, 0);
    joint->SetLowStop(0, 0);
    //joint->SetParam("cfm", 0, 0);
    link_object_map.erase(link_name);
    collisions_str.clear();//TODO better
    return true;
}

void GazeboYarpObjects::OnContacts(ConstContactsPtr& iter)
{
    std::vector<std::string>::iterator collIter, objectIter;
    std::string collision, object;
    {
        // Iterate over all the contacts in the message
        for (int i = 0; i < (iter)->contact_size(); ++i)
        {
            collision = (iter)->contact(i).collision1();
            // Try to find the first collision's name
            collIter = std::find(this->collisions_str.begin(),
                                this->collisions_str.end(), collision);
            
            // If unable to find the first collision's name, try the second
            if (collIter == this->collisions_str.end())
            {
                object = (iter)->contact(i).collision1();
                collision = (iter)->contact(i).collision2();
                collIter = std::find(this->collisions_str.begin(),
                                    this->collisions_str.end(), collision);
            }
            else
            object = (iter)->contact(i).collision2();
            // If this sensor is monitoring one of the collision's in the
            // contact, then add the contact to our outgoing message.
            if (collIter != this->collisions_str.end())
            {
                int count = (iter)->contact(i).position_size();

                // Check to see if the contact arrays all have the same size.
                if (count != (iter)->contact(i).normal_size() ||
                    count != (iter)->contact(i).wrench_size() ||
                    count != (iter)->contact(i).depth_size())
                {
                    gzerr << "Contact message has invalid array sizes\n";
                    continue;
                }
                for (std::map<std::string,std::string>::iterator it=link_object_map.begin();it!=link_object_map.end();++it)
                {
                    if (object.find((it->second),0)==0)
                    {
                        std::cout<<(iter)->contact(i).position(0).x()<<(iter)->contact(i).position(0).y()<<(iter)->contact(i).position(0).z()<<std::endl;
                        gazebo::math::Pose touch_point;
                        touch_point.pos.x=(iter)->contact(i).position(0).x();
                        touch_point.pos.y=(iter)->contact(i).position(0).y();
                        touch_point.pos.z=(iter)->contact(i).position(0).z();
                        math::Vector3 normal;
                        normal.x=(iter)->contact(i).position(0).x();
                        normal.y=(iter)->contact(i).position(0).y();
                        normal.z=(iter)->contact(i).position(0).z();
                        attach_impl(it->first,it->second,touch_point,normal);
                    }
                }
            }
        }
    }
}


bool gazebo::GazeboYarpObjects::detach(const std::string& object_name)
{
    physics::JointPtr joint, handle_joint;
    if (joints_attached.count(object_name+"_attached_joint"))
    {
        joint=joints_attached[object_name+"_attached_joint"];
    }
    else
    {
        return false;
    }
    if (joints_attached.count(object_name+"_attached_handle_joint"))
    {
        handle_joint=joints_attached[object_name+"_attached_handle_joint"];
    }
    else
    {
        return false;
    }
    physics::ModelPtr object_model = m_world->GetModel(object_name);
    if( !object_model )
    {
        std::cout<<"could not get the model for "<<object_name<<"!!"<<std::endl;
        return false;
    }

    //TODO add mutex
    joint->Detach();
    joints_attached.erase(object_name+"_attached_joint");
    handle_joint->Detach();
    joints_attached.erase(object_name+"_attached_handle_joint");
    //TODO: enable collisions on the hand, not on the object
    gazebo::physics::Link_V model_links = object_model->GetLinks();
    for(int i=0; i < model_links.size(); i++ ) 
    {
        model_links[i]->SetCollideMode("all");
    }
    return true;
}


}