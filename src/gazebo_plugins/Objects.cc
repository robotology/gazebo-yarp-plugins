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
        std::string candidate_link = model_links[i]->GetName();
        if( hasEnding(candidate_link,"::"+link_name) ) {
            return model_links[i];
        }
    }

    return gazebo::physics::LinkPtr();
}

bool gazebo::GazeboYarpObjects::attach(const std::string& link_name, const std::string& object_name)
{
    physics::JointPtr joint;
    joint = m_world->GetPhysicsEngine()->CreateJoint("revolute", m_model);

    if( !joint ) {
        return false;
    }

    physics::ModelPtr object_model = m_world->GetModel(object_name);

    if( !object_model )
    {
        return false;
    }

    physics::LinkPtr object_link = object_model->GetLink();
    physics::LinkPtr parent_link = getLinkInModel(m_model,link_name);

    if( !object_link ||
        !parent_link ) {
        return false;
    }

    math::Pose parent_link_pose = parent_link->GetWorldCoGPose();
    object_link->SetWorldPose(parent_link_pose);

    //TODO add mutex
    joint->Load(parent_link, object_link, math::Pose());
    joint->Attach(parent_link, object_link);
    joint->SetHighStop(0, 0);
    joint->SetLowStop(0, 0);
    //joint->SetParam("cfm", 0, 0);

    return true;
}

}