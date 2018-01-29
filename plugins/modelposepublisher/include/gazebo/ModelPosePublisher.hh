/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_MODELPOSEPUBLISHER_HH
#define GAZEBOYARP_MODELPOSEPUBLISHER_HH

// gazebo
#include <gazebo/common/Plugin.hh>

// yarp
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/PolyDriver.h>

namespace gazebo
{
    /// \class GazeboYarpModelPosePublisher
    /// Gazebo Plugin that publishes the pose of the root link
    /// of a model with respect to the Gazebo world frame.
    ///
    /// This plugin instantiates a `yarp::dev::FrameTransformClient`
    /// in order to publish the transform between the Gazebo world frame
    /// and the root link of the model.
    /// This requires a `yarp::dev::FrameTransformServer` to be actived and
    /// running before the model is inserted in Gazebo. It is not required to
    /// write code in order to have a working `FrameTransfomServer`. Instead `yarpdev`
    /// can be used from the command line, e.g.
    /// \code
    /// yarpdev --device transformServer --ROS::enable_ros_publisher false --ROS::enable_ros_subscriber false
    /// \endcode
    /// In the example above support to ROS is disabled.
    ///
    /// Each transform published by the plugin contains as `source` frame the string
    /// "/inertial" and as `target` frame a string that depends on the `name` option
    /// of the `model` tag within the SDF. If such a name is `<name>` the string describing
    /// the `target` will be "/<name>/frame". In case the model `<name>` is inserted in the
    /// Gazebo environment more than once than the its name becomes `<name>_x` where
    /// `x` assumes the values `0, 1, 2...`. This beavior depends on the internal behavior of
    /// Gazebo.
    ///
    /// The pose published by the plugin is that of the __root link__ of the model with respect
    /// to the Gazebo world frame. If the SDF of the model is the following
    /// \code
    /// <sdf>
    ///   <model name="object">
    ///   <!-- pose of model frame w.r.t Gazebo world -->
    ///   <pose> x_m y_m z_m roll_m pich_m yaw_m </pose>
    ///
    ///     <link name="object_root_link">
    ///     <!-- pose of the root link w.r.t the model frame -->
    ///     <pose> x_l y_l z_l roll_l pich_l yaw_l </pose>    
    ///     ...
    ///     </link>
    ///
    ///    <plugin name='...' filename='libgazebo_yarp_modelposepublisher.so'>
    ///      <period>1.0</period>
    ///    </plugin>
    ///
    ///	  </model>
    /// <\sdf>
    /// \endcode
    /// then the pose of the root link depends on both the transformation between
    /// the Gazebo world frame and the model frame and the transformation between
    /// the model frame and the root link.
    ///
    /// In order to retrieve the published transform an istance of
    /// `yarp::dev::FrameTransformClient` can be used. An example is provided below
    /// \code
    /// ...
    /// // Instantiate the driver
    ///	yarp::dev::PolyDriver m_drvTransformClient;
    /// yarp::dev::IFrameTransform* m_tfClient;
    /// yarp::os::Property propTfClient;
    /// propTfClient.put("device", "transformClient");
    /// propTfClient.put("local", "/transformClientLocal");
    /// propTfClient.put("remote", "/transformServer");
    ///
    /// // Check if the driver opened correctly
    /// bool ok_open = m_drvTransformClient.open(propTfClient);
    /// if (!ok) {
    ///     // Error...
    /// }
    ///
    /// // Check if the view was obtained succesfully
    /// bool ok_view = m_drvTransformClient.view(m_tfClient);
    /// if (!ok_view || m_tfClient == nullptr) {
    ///     // Error...
    /// }
    ///
    /// // Retrieve the transform
    /// yarp::sig::Matrix transform(4, 4);
    /// std::string source = "/inertial";
    /// std::string target = "/<name>/frame";
    /// bool ok_transform = m_tfClient->getTransform(target, source, transform);
    /// if (!ok_transform) {
    ///     // Transform not ready...
    /// }
    /// \endcode
    ///
    /// The update rate of the plugin can be configured using the `period`
    /// SDF tag in seconds. If the parameter is not provided a default value of 10 ms
    /// is used.
    ///
    /// This plugin __does not__ support [Gazebo nested models](http://gazebosim.org/tutorials?tut=nested_model&cat=build_robot)
    /// since the method `gazebo::physics::WorldPose()` does not work properly
    /// with nested models (see [issue](https://bitbucket.org/osrf/gazebo/issues/2410/wrong-gazebo-physics-model-worldpose-for")).
    ///
    class GazeboYarpModelPosePublisher : public ModelPlugin
    {
    public:
	~GazeboYarpModelPosePublisher();
	
	/**
	 * Store pointer to the model, load parameters from the SDF,
	 * configure the frame transform client, reset the time of the last update and
	 * connect to the World update event of Gazebo.
	 */	
	void Load(gazebo::physics::ModelPtr, sdf::ElementPtr);
	
    private:
	/**
	 * Instance of yarp::os::Network
	 */
	yarp::os::Network m_yarp;

	/**
	 * Pointer to the model where the plugin is inserted
	 */
	gazebo::physics::ModelPtr m_model;

	/**
	 * Connection to the World update event of Gazebo
	 */
	gazebo::event::ConnectionPtr m_worldUpdateConnection;

	/**
	 * Time of the last update of the plugin
	 */
	gazebo::common::Time m_lastUpdateTime;
		
	/**
	 * Update period of the plugin
	 */
	double m_period;

	/**
	 * PolyDriver required to access a yarp::dev::IFrameTransform
	 */
	yarp::dev::PolyDriver m_drvTransformClient;

	/**
	 * Pointer to yarp::dev::IFrameTransform view of the PolyDriver
	 */
	yarp::dev::IFrameTransform* m_tfClient;

	/**
	 * Publish the transform from the inertial frame to the model root link frame.
	 */	
	void PublishTransform();

	/**
	 * Check if a period is elapsed since last update 
	 * and in case calls the method `PublishTransform`.
	 */	
	void OnWorldUpdate();
    };
}
#endif
