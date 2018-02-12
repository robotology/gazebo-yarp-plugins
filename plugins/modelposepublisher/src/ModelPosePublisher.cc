/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

// gazebo
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/World.hh>

// ignition
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

// GazeboYarpPlugins
#include <GazeboYarpPlugins/common.h>

// yarp
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/math/FrameTransform.h>

// boost
#include <boost/bind.hpp>

#include "ModelPosePublisher.hh"

GZ_REGISTER_MODEL_PLUGIN(gazebo::GazeboYarpModelPosePublisher)

namespace gazebo {

GazeboYarpModelPosePublisher::~GazeboYarpModelPosePublisher()
{
    // Close the driver
    m_drvTransformClient.close();
}

void GazeboYarpModelPosePublisher::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Check yarp network availability
    if (!m_yarp.checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
        yError() << "GazeboYarpModelPosePublisher::Load error:"
		 << "yarp network does not seem to be available, is the yarpserver running?";
        return;
    }

    // Store pointer to the model
    m_model = _parent;

    // Load update period
    if (_sdf->HasElement("period")) {
	// Get the parameter
	sdf::ParamPtr periodPtr = _sdf->GetElement("period")->GetValue();

	// Check if it is a strictly positive double
	if (!periodPtr->Get<double>(m_period) || m_period <= 0) {
		yError() << "GazeboYarpModelPosePublisher::Load error:"
			 << "parameter 'period' for model"
			 << m_model->GetName() << "should be a strictly positive number";
		return;
	}
    } else {
	// Default to 10 ms
	m_period = 0.01;
    }

    // Prepare properties for the PolyDriver
    yarp::os::Property propTfClient;    
    propTfClient.put("device", "transformClient");
    // The local port depends on the model name that is unique
    // also in the case of multiple insertions of the same model
    // in Gazebo
    propTfClient.put("local", "/" + m_model->GetName() + "/transformClient");
    propTfClient.put("remote", "/transformServer");
    // This is the update period of the transformClient in ms
    propTfClient.put("period", m_period * 1000);

    // Open the driver and obtain a a IFrameTransform view
    m_tfClient = nullptr;    
    bool ok = m_drvTransformClient.open(propTfClient);
    ok = ok && m_drvTransformClient.view(m_tfClient) && m_tfClient != nullptr;
    
    // Return if the driver open failed
    // or the view retrieval failed
    // or the IFrameTransform pointer is not valid
    if (!ok) {
	yError() << "GazeboYarpModelPosePublisher::Load error:"
	         << "failure in opening iFrameTransform interface for model"
		 << m_model->GetName();
	return;
    }
    
    // Clear last update time
    m_lastUpdateTime = gazebo::common::Time(0.0);

    // Listen to the update event
    auto worldUpdateBind = boost::bind(&GazeboYarpModelPosePublisher::OnWorldUpdate, this);
    m_worldUpdateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(worldUpdateBind);
}

void GazeboYarpModelPosePublisher::PublishTransform()
{
    // Get the current pose of the canonical link of the model    
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d curPose = m_model->GetLink()->WorldPose();
#else
    gazebo::math::Pose curPoseGazebo = m_model->GetLink()->GetWorldPose();
    // Convert to Ignition so that the same interface
    // can be used in the rest of the function
    ignition::math::Pose3d curPose = curPoseGazebo.Ign();
#endif

    // Get the positional and rotational parts
    ignition::math::Vector3d pos = curPose.Pos();
    ignition::math::Quaterniond rot = curPose.Rot();
 
    // Convert the pose to a homogeneous transformation matrix
    // (the first two arguments can be blank)
    yarp::math::FrameTransform  inertialToModel("", "",
						pos.X(),
						pos.Y(),
						pos.Z(),
						rot.X(),						
						rot.Y(),
						rot.Z(),
						rot.W());

    // Set a new transform using
    // /inertial for source frame name and
    // /<model_name>/frame for the target frame name
    m_tfClient->setTransform("/" + m_model->GetName() + "/frame",
			     "/inertial",
			     inertialToModel.toMatrix());
}

void GazeboYarpModelPosePublisher::OnWorldUpdate()
{    
    // Get current time
#if GAZEBO_MAJOR_VERSION >= 8
    gazebo::common::Time currentTime = m_model->GetWorld()->SimTime();
#else
    gazebo::common::Time currentTime = m_model->GetWorld()->GetSimTime();
#endif
    
    // Check if an update period is elapsed
    if(currentTime - m_lastUpdateTime >= m_period) {
	
	// Store current time for next update
	m_lastUpdateTime = currentTime;

	// Publish the transform with the current pose
	PublishTransform();
    }
}

}
