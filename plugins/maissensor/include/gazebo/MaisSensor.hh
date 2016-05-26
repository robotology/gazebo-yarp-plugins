/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_MAISSENSOR_HH
#define GAZEBOYARP_MAISSENSOR_HH

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
/// \class GazeboYarpMaisSensor
/// Gazebo Plugin emulating the yarp mais sensor (IAnalogSensor) in Gazebo.
///.
class GazeboYarpMaisSensor : public ModelPlugin
{
public:
    GazeboYarpMaisSensor();
    virtual ~GazeboYarpMaisSensor();

    /**
     * Saves the gazebo pointer, creates the device driver
     */
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

private:
    yarp::dev::PolyDriver m_wrapper;
    yarp::dev::IMultipleWrapper* m_iWrap;

    yarp::os::Property m_parameters;
    std::string m_sensorName;
    std::string m_robotName;
};

}

#endif
