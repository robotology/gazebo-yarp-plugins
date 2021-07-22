/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia - iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

/**
 * @file DoubleLaser.hh
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#ifndef GAZEBOYARP_DOUBLELASER_HH
#define GAZEBOYARP_DOUBLELASER_HH

#include <gazebo/common/Plugin.hh>

#include <string>

#include <yarp/dev/PolyDriverList.h>

namespace yarp {
    namespace dev {
        class IMultipleWrapper;
    }
}

namespace gazebo
{
/// \class GazeboYarpDoubleLaser
/// Gazebo Plugin emulating the yarp double laser device in Gazebo.
///.
/// It can be configured using the yarpConfigurationFile sdf tag,
/// that contains a Gazebo URI pointing at a yarp .ini configuration file
/// containing the configuration parameters of the DoubleLaser
///


class GazeboYarpDoubleLaser : public ModelPlugin
{
public:
    GazeboYarpDoubleLaser();
    virtual ~GazeboYarpDoubleLaser();

    /**
     * Saves the gazebo pointer, creates the device driver
     */
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

private:

    bool readConfigurationFromFile(physics::ModelPtr _parent, sdf::ElementPtr _sdf); //Getting .ini configuration file from sdf
    #ifndef USE_NEW_WRAPPERS
    yarp::dev::PolyDriver m_wrapper_rangeFinder;
    yarp::dev::IMultipleWrapper* m_iWrap_rangeFinder;
    #endif
    yarp::dev::PolyDriver m_driver_doublelaser;
    yarp::dev::IMultipleWrapper* m_iWrap_doublelaser;

    yarp::dev::PolyDriverList m_lasers;  //contains pointers of front and back laser

    yarp::os::Property m_parameters;

    std::string m_sensorName;

    yarp::dev::PolyDriver * m_driver_laserFront;
    yarp::dev::PolyDriver * m_driver_laserBack;
};

}

#endif
