/*
 * Copyright (C) 2007-2015 Istituto Italiano di Tecnologia RBCS, ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
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

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/Network.h>

#include <sstream>

GZ_REGISTER_MODEL_PLUGIN(gazebo::GazeboYarpObjects)

namespace gazebo {

GazeboYarpObjects::GazeboYarpObjects() : ModelPlugin(), m_network()
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

bool GazeboYarpObjects::deleteObject(const std::string& object_name)
{
    if (m_world->GetModel(object_name)!=NULL)
    {
        m_world->RemoveModel(object_name);
        return true;
    }
    else
        return false;
}


bool gazebo::GazeboYarpObjects::createSphere(const std::string& name, const double radius, const double mass)
{
    /** Remove collision to facilitate hand:
     *    <collision name ='collision'>
                <geometry>
                  <sphere><radius>"<< radius << "</radius></sphere>
                </geometry>
              </collision> */

    sdf::SDF sphereSDF;
    std::stringstream ss;
    ss << "<sdf version ='1.4'>\
            <model name ='" << name << "'>\
            <pose>1 0 0 0 0 0</pose>\
            <link name ='" << name << "_link'>\
              <pose>0 0 .5 0 0 0</pose>\
              <visual name ='visual'>\
                <geometry>\
                  <sphere><radius>"<< radius << "</radius></sphere>\
                </geometry>\
              </visual>\
              <material>\
              <script>\
              <uri>file://media/materials/scripts/gazebo.material</uri>\
              <name>Gazebo/Grey</name>\
              </script>\
              </material>\
               <inertial>\
                 <pose>0 0 0 0 0 0</pose>\
                 <mass>" << mass << "</mass>\
                 <inertia>\
                   <ixx>" << 2.0*radius*radius*mass/5.0 << "</ixx>\
                   <ixy>0</ixy>\
                   <ixz>0</ixz>\
                   <iyy>" << 2.0*radius*radius*mass/5.0 << "</iyy>\
                   <iyz>0</iyz>\
                   <izz>"<< 2.0*radius*radius*mass/5.0 << "</izz>\
                 </inertia>\
               </inertial>\
            </link>\
          </model>\
        </sdf>";
    std::string sphereSDF_str = ss.str();
    sphereSDF.SetFromString(sphereSDF_str);
    m_world->InsertModelSDF(sphereSDF);

    return true;
}

bool hasEnding (std::string const &fullString, std::string const &ending)
{
    std::cout<<fullString<<" "<<ending<<std::endl;
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
{    std::cout<<"called"<<std::endl;
    
    gazebo::physics::Link_V model_links = model->GetLinks();
    for(int i=0; i < model_links.size(); i++ ) 
    {
        std::string candidate_link = model_links[i]->GetScopedName();
        std::cout<<candidate_link<<std::endl;
        double norm = pose.pos.Distance(model_links[i]->GetWorldPose().pos);
        if (norm<0.15)
        {
            //         std::string candidate_link = model_links[i]->GetScopedName();
            //         std::cout<<candidate_link<<std::endl;
            return model_links[i];
        }
    }
    return gazebo::physics::LinkPtr();
}

bool gazebo::GazeboYarpObjects::attach(const std::string& link_name, const std::string& object_name)
{    std::cout<<"called"<<std::endl;
    
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

    physics::LinkPtr parent_link = getLinkInModel(m_model,link_name);

    if( !parent_link ) {
        std::cout<<"could not get the links for "<<parent_link<<std::endl;
        return false;
    }

    physics::LinkPtr object_link = getClosestLinkInModel(object_model,parent_link->GetWorldCoGPose());
    if( !object_link ) {
        std::cout<<"could not get a link close enough (<5cm) to "<<parent_link<<" for object "<<object_name<<std::endl;
        return false;
    }

    math::Pose parent_link_pose = parent_link->GetWorldCoGPose();
    object_link->SetWorldPose(parent_link_pose);

    //TODO add mutex
    physics::JointPtr joint;
    joint = m_world->GetPhysicsEngine()->CreateJoint("revolute", m_model);
    if( !joint ) {
        std::cout<<"could not create joint!!"<<std::endl;
        return false;
    }
    joint->SetName(object_name+"_attached_joint");
    joints_attached[object_name+"_attached_joint"]=joint;
    joint->Load(parent_link, object_link, math::Pose());
    joint->Attach(parent_link, object_link);
    joint->SetHighStop(0, 0);
    joint->SetLowStop(0, 0);
    //joint->SetParam("cfm", 0, 0);

    return true;
}

bool gazebo::GazeboYarpObjects::detach(const std::string& object_name)
{    std::cout<<"called"<<std::endl;
    
    physics::JointPtr joint;
    joint=joints_attached[object_name+"_attached_joint"];

    if( !joint ) {
        return false;
    }

    //TODO add mutex
    joint->Detach();
    joints_attached.erase(object_name+"_attached_joint");
    return true;
}


}