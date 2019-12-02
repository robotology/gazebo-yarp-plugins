/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBO_YARP_CONTACTFORCETORQUES_HH
#define GAZEBO_YARP_CONTACTFORCETORQUES_HH

#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/Link.hh>

#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <yarp/sig/Vector.h>

namespace gazebo
{
    /**
     * \class GazeboYarpContactForceTorques
     * Gazebo plugin simulating the estimation of contact force torques
     *.
     * This plugin does not simulate any physical sensor, but it simulates
     * the output of the estimators of external contact force torques,
     * similarly to how the gazebo_yarp_basestate plugin simulates the output
     * of floating base position estimators.
     *
     * On real robot, the contact forcetorques are estimated by a dedicated
     * software based on the available sensor measurements (such as motor currents,
     * six axis force-torque sensors measures and distribute tactile sensor measurements)
     * and/or a-priori assumptions on the location of the contact (for example on
     * industrial manipulators the external contact is usually assume to be only possible
     * on the end-effector). For example, on the iCub robot the wholebodydynamics software
     * is responsible for such an estimation.
     * In a simulation, it is possible to obtain directly this information
     * for the simulator state and inputs, if it is necessary to have a perfect estimate.
     *
     * Differntly for other kind of functionalities such as sensor or  motor interfaces,
     * at the moment YARP does not provide any kind of standard interface of message to represent
     * contact force torques (see https://github.com/robotology/yarp/issues/2134 for more details),
     * so at the moment the plugin only permits to stream the total contact force torque being applied
     * on a given link on  a single YARP port that streams a yarp::sig::Vector, similarly to how the
     * wholebodydynamics software streams contact information. The first three elements represent the force
     * expressed in N, while the last three elements the torque expressed in Nm. The forcetorque estimated is the
     * one applied on the link.
     *
     * This plugin can be used by adding the following element as a child of your sdf model tag:
     *  ```
     *      <plugin name="contactforcetorques_instance" filename="libgazebo_yarp_contactforcetorque.so">
     *          <yarpConfigurationFile>model://path-to-the-configuration-file</yarpConfigurationFile>
     *      </plugin>
     *  ```
     *
     * The plugin .ini configuration file must contain a parameter called periodInSeconds, and a group called [OUTPUT_EXTERNAL_FORCETORQUES_PORTS],
     * and may contain a group called [OUTPUT_EXTERNAL_FORCETORQUES_FRAMES] .
     *
     * The periodInSeconds parameter is required, and represent the publishing period in seconds.
     *
     * The OUTPUT_EXTERNAL_FORCETORQUES_PORTS specifies the port that should be opened by the plugin, with the following format:
     *
     * | Parameter name| Type |Units|Default Value|Required |                 Description                |              Notes             |
     * |:-------------:|:----:|:---:|:-----------:|:-------:|:------------------------------------------:|:------------------------------:|
     * |   portName_1   |  The name of the parameter specifies the port name, while the value is  Bottle of three elements describing the force torque published on the port. | - | - | No    | Bottle of three elements describing the wrench published on the port: the first element is the link of which the published external forcetorque is applied. This forcetorque is expressed around the origin of the frame named as second parameter, and with the orientation of the third parameter.  |  |
     * | ...  | ... | - | ..                                        | No       | ..  |  |
     * | portName_n  | ... | - | ..                                        | No       | ..  |  |
     *
     *
     * The OUTPUT_EXTERNAL_FORCETORQUES_PORTS
     * group has a similar format to the WBD_OUTPUT_EXTERNAL_WRENCH_PORTS of wholebodydynamics (document in
     *  https://github.com/robotology/codyco-modules/blob/master/src/devices/wholeBodyDynamics/WholeBodyDynamicsDevice.h#L139 ).
     *
     * While the first parameter of the bottle of each line of OUTPUT_EXTERNAL_FORCETORQUES_PORTS identifies a link,
     * the second and the third can also just refer to a additional frame of a link. Unfortunatly, additional frames are removed
     * in the conversion of URDF to SDF, and in general are not contained in SDF version <= 1.6 . For this reason, you can
     * specify additional frames directly in the configuration file of this plugin using the `OUTPUT_EXTERNAL_FORCETORQUES_FRAMES` group, with the following format:
     *
     * | Parameter name| Type |Units|Default Value|Required |                 Description                |              Notes             |
     * |:-------------:|:----:|:---:|:-----------:|:-------:|:------------------------------------------:|:------------------------------:|
     * |   additionalFrameName_1   |  The name of the parameter specifies the additional frame name, while the value is  Bottle of seven elements describing the . | - | - | No    | Bottle of seven elements elements describing the additional frame: the first element is the link to which the additional frame is attached, the rest of the elements specify the link_H_additionalFrame transfrom in the SDF convention. |  |
     * | ...  | ... | - | ..                                        | No       | ..  |  |
     * | additionalFrameName_n  | ... | - | ..                                        | No       | ..  |  |
     *
     * An example configuration file might look like:
     *
     * ```
     * periodInSeconds 0.01
     *
     * [OUTPUT_EXTERNAL_FORCETORQUES_PORTS]
     * /wholeBodyDynamics/left_foot/cartesianEndEffectorWrench:o (l_foot,l_sole,l_sole)
     * /wholeBodyDynamics/right_foot/cartesianEndEffectorWrench:o (r_foot,r_sole,r_sole)
     *
     * [OUTPUT_EXTERNAL_FORCETORQUES_FRAMES]
     * l_sole (l_foot,0.0035,0.0,0.004,-3.14159265359,0.0,0.0)
     * r_sole (r_foot,0.0035,0.0,0.004,-3.14159265359,0.0,0.0)
     *
     * ```
     */
    class GazeboYarpContactForceTorques : public ModelPlugin
    {
    public:
        GazeboYarpContactForceTorques();
        virtual ~GazeboYarpContactForceTorques();
        
        /**
         * Loads robot model, reads configuration, 
         * opens network wrapper device and opens device driver
         */
        void Load(physics::ModelPtr model, sdf::ElementPtr _sdf);

        /**
         * Callback for the WorldUpdateBegin Gazebo event.
         */
        void onUpdate(const gazebo::common::UpdateInfo&);

        /**
         *
         */
        void onReset();
    private:
        struct Pimpl;
        struct AdditionalFrameInformation;
        struct OutputWrenchPortInformation;
        struct ForceTorque;

        /**
         * Load and validate parameters.
         */
        bool LoadParams(physics::ModelPtr model);

        /**
         * Load and validate parameters related to additional frames.
         */
        bool LoadAdditionalFrameParams(physics::ModelPtr model);

        /**
         * Load and validate parameters related to streaming ports.
         */
        bool LoadStreamingPortParams(physics::ModelPtr model);

        /**
         * Open YARP ports.
         */
        bool openPorts();

        /**
         * Close yarp ports.
         */
        bool closePorts();

        /**
         * Change the forcetorque expressed in a frame to be expressed (both origin and orientation) in another frame.
         */
        GazeboYarpContactForceTorques::ForceTorque applyTransformToForceTorque(ignition::math::Pose3d& newFrame_H_oldFrame,
                                                                                GazeboYarpContactForceTorques::ForceTorque&  ft_oldFrame);

        /**
         * Convert a GazeboYarpContactForceTorques::ForceTorque to a YARP vector.
         */
        void toYARP(GazeboYarpContactForceTorques::ForceTorque& ft_gazebo,
                    yarp::sig::Vector& ft_yarp);

        /**
         * Get total force that the enviroment applies on a link, expressed in the link frame (both origin and orientation)
         */
        GazeboYarpContactForceTorques::ForceTorque getTotalForceTorqueAppliedOnLink(gazebo::physics::LinkPtr link);

        std::unique_ptr<Pimpl> m_pimpl;
    };
}

#endif

