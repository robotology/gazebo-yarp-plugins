/*
 * Copyright (C) 2013-2015 Istituto Italiano di Tecnologia RBCS & ADVR & iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "ShowModelCoM.hh"

#include <iostream>
#include <string>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/Node.hh>

#include <yarp/os/Network.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>


namespace gazebo
{

    GZ_REGISTER_MODEL_PLUGIN (ShowModelCoM)

    ShowModelCoM::ShowModelCoM()
    : m_comOutputPort(0)
    {
        yarp::os::Network::init();
    }

    ShowModelCoM::~ShowModelCoM()
    {
        if (m_comOutputPort) {
            m_comOutputPort->interrupt();
            m_comOutputPort->close();
            delete m_comOutputPort;
            m_comOutputPort = 0;
        }
        gazebo::event::Events::DisconnectWorldUpdateBegin(this->m_updateConnection);
        yarp::os::Network::fini();
    }

    void ShowModelCoM::UpdateChild()
    {
        gazebo::physics::Link_V links = m_myModel->GetLinks();

        double mass_acc = 0.0;
        gazebo::math::Vector3 weighted_position_acc = gazebo::math::Pose::Zero.pos;
        for(unsigned int i = 0; i < links.size(); ++i)
        {
            gazebo::physics::LinkPtr link = links[i];
            gazebo::math::Vector3 wordlCoG_i = link->GetWorldCoGPose().pos;
            double mass_i = link->GetInertial()->GetMass();

            weighted_position_acc += mass_i*wordlCoG_i;
            mass_acc += mass_i;
        }


        gazebo::math::Vector3 wordlCoGModel = weighted_position_acc/mass_acc;

        if (m_comOutputPort) {
            yarp::os::Bottle &bot = m_comOutputPort->prepare();
            bot.clear();
            bot.addDouble(wordlCoGModel.x);
            bot.addDouble(wordlCoGModel.y);
            bot.addDouble(wordlCoGModel.z);
            m_comOutputPort->write();
        }

        gazebo::math::Pose WorldCoGPose = gazebo::math::Pose::Zero;
        WorldCoGPose.pos = wordlCoGModel;

#if GAZEBO_MAJOR_VERSION >= 7
        msgs::Set(m_visualMsg.mutable_pose(), WorldCoGPose.Ign());
#else
        msgs::Set(m_visualMsg.mutable_pose(), WorldCoGPose);
#endif
        msgs::Set(m_visualMsg.mutable_material()->mutable_ambient(),common::Color(1,0,0,0.3));
        m_visualMsg.set_visible(1);
        m_visPub->Publish(m_visualMsg);
    }

    void ShowModelCoM::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // Check if yarp network is active;
        if (!yarp::os::Network::checkNetwork()) {
            yError("ERROR Yarp Network was not found active");
            return;
        }

        // What is the parent name??
        this->m_modelScope = _model->GetScopedName();

        std::string port_name = "/" + this->m_modelScope + "/CoMInWorld:o";
        m_comOutputPort = new yarp::os::BufferedPort<yarp::os::Bottle>();
        if (!m_comOutputPort || !m_comOutputPort->open(port_name)) {
            yError("Could not open port %s", port_name.c_str());
            return;
        }

        // Copy the pointer to the model to access later from UpdateChild
        this->m_myModel = _model;
        /// ############ TRYING TO MODIFY VISUALS #######################################################

        this->m_node = transport::NodePtr(new gazebo::transport::Node());

        this->m_node->Init ( _model->GetWorld()->GetName() );
        m_visPub = this->m_node->Advertise<msgs::Visual> ("~/visual", 10);

        // Set the visual's name. This should be unique.
        m_visualMsg.set_name ("__COM_VISUAL__");

        // Set the visual's parent. This visual will be attached to the parent
        m_visualMsg.set_parent_name ( _model->GetScopedName() );

        // Create a cylinder
        msgs::Geometry *geomMsg = m_visualMsg.mutable_geometry();
        geomMsg->set_type ( msgs::Geometry::SPHERE );
        geomMsg->mutable_sphere()->set_radius ( 0.02 );

        // Don't cast shadows
        m_visualMsg.set_cast_shadows ( false );

        /// ############ END: TRYING TO MODIFY VISUALS #####################################################

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->m_updateConnection = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &ShowModelCoM::UpdateChild, this ) );
    }
}
