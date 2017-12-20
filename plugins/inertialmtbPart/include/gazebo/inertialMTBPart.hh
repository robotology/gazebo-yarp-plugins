/*
 * Copyright (C) 2013-2017 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#ifndef GAZEBOYARP_INERTIALMTBPART_HH
#define GAZEBOYARP_INERTIALMTBPART_HH

#include <gazebo/common/Plugin.hh>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>

#include <string>


namespace gazebo
{
    namespace sensors {
        class ImuSensor;
    }

    /// \class GazeboYarpInertialMTBPart
    /// Gazebo Plugin emulating the yarp inertial MTB sensors device in Gazebo.
    ///
    /// This plugin instantiate a yarp MTB board driver for the Gazebo simulator
    /// and instantiate a network wrapper (provided by yarp::dev::AnalogWrapper)
    /// to expose the sensors of a whole model part (ex: the leg) on the yarp network.
    /// Since a part can include several links (ex: upper_leg, lower_leg and
    /// foot), the plugin is tied to the model.
    ///
    /// It can be configurated using the yarpConfigurationFile sdf tag,
    /// that contains a Gazebo URI pointing at a yarp .ini configuration file
    /// containing the configuration parameters of the controlBoard
    ///
    /// The parameter that the yarpConfigurationFile must contain are:
    ///  <TABLE>
    ///  <TR><TD> name </TD><TD> Port name to assign to the wrapper to this device. </TD></TR>
    ///  <TR><TD> period </TD><TD> Update period (in s) of yarp port that publish the measure. </TD></TR>
    ///  </TABLE>
    /// If the required parameters are not specified, their value will be the
    /// default one assigned by the yarp::dev::AnalogWrapper wrapper .
    ///
    class GazeboYarpInertialMTBPart : public ModelPlugin
    {
    public:
        GazeboYarpInertialMTBPart();
        virtual ~GazeboYarpInertialMTBPart();
        /**
         * Saves the gazebo pointer, retrieves the configuration parameters from the sdf
         * file and creates the device driver
         */
        virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    private:
        yarp::dev::PolyDriver m_inertialMTBwrapper;
        yarp::dev::IMultipleWrapper* m_iWrap;
        yarp::dev::PolyDriver m_inertialMTBpartDriver;
        std::string m_robotName;
    };
}

#endif // GAZEBOYARP_INERTIALMTBPART_HH
