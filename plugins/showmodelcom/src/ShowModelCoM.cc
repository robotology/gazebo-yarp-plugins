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

        m_updateConnection.reset();
        yarp::os::Network::fini();
    }

    void ShowModelCoM::UpdateChild()
    {
        gazebo::physics::Link_V links = m_myModel->GetLinks();

        double mass_acc = 0.0;
#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Vector3d weighted_position_acc = ignition::math::Vector3d::Zero;
#else
        gazebo::math::Vector3 weighted_position_acc = gazebo::math::Pose::Zero.pos;
#endif
        for(unsigned int i = 0; i < links.size(); ++i)
        {
            gazebo::physics::LinkPtr link = links[i];
#if GAZEBO_MAJOR_VERSION >= 8
            ignition::math::Vector3d wordlCoG_i = link->WorldCoGPose().Pos();
#else
            gazebo::math::Vector3 wordlCoG_i = link->GetWorldCoGPose().pos;
#endif

#if GAZEBO_MAJOR_VERSION >= 8
            double mass_i = link->GetInertial()->Mass();
#else
            double mass_i = link->GetInertial()->GetMass();
#endif

            weighted_position_acc += mass_i*wordlCoG_i;
            mass_acc += mass_i;
        }

#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Vector3d wordlCoGModel = weighted_position_acc/mass_acc;
#else
        gazebo::math::Vector3 wordlCoGModel = weighted_position_acc/mass_acc;
#endif

        if (m_comOutputPort) {
            yarp::os::Bottle &bot = m_comOutputPort->prepare();
            bot.clear();
#if GAZEBO_MAJOR_VERSION >= 8
            bot.addFloat64(wordlCoGModel.X());
            bot.addFloat64(wordlCoGModel.Y());
            bot.addFloat64(wordlCoGModel.Z());
#else
            bot.addFloat64(wordlCoGModel.x);
            bot.addFloat64(wordlCoGModel.y);
            bot.addFloat64(wordlCoGModel.z);
#endif
            m_comOutputPort->write();
        }

#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d WorldCoGPose = ignition::math::Pose3d::Zero;
        WorldCoGPose.Pos() = wordlCoGModel;
#else
        gazebo::math::Pose WorldCoGPose = gazebo::math::Pose::Zero;
        WorldCoGPose.pos = wordlCoGModel;
#endif

#if GAZEBO_MAJOR_VERSION == 7
        msgs::Set(m_visualMsg.mutable_pose(), WorldCoGPose.Ign());
#else
        msgs::Set(m_visualMsg.mutable_pose(), WorldCoGPose);
#endif

        double red = 1;
        double alpha = 0.3;
#if GAZEBO_MAJOR_VERSION >= 9
        msgs::Set(m_visualMsg.mutable_material()->mutable_ambient(), ignition::math::Color(red,0,0,alpha));
#else
        msgs::Set(m_visualMsg.mutable_material()->mutable_ambient(), common::Color(red,0,0,alpha));
#endif
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

#if GAZEBO_MAJOR_VERSION >= 8
        std::string worldName = _model->GetWorld()->Name();
#else
        std::string worldName = _model->GetWorld()->GetName();
#endif
        this->m_node->Init ( worldName );
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
