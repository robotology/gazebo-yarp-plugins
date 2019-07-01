/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Lorenzo Rapetti.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_CONFIGURATIONOVERRIDE_HH
#define GAZEBOYARP_CONFIGURATIONOVERRIDE_HH

#include <gazebo/common/Plugin.hh>
#include <GazeboYarpPlugins/common.h>

namespace gazebo
{
class ConfigurationOverride : public ModelPlugin
{
public:
    ConfigurationOverride();
    virtual ~ConfigurationOverride();

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

private:
};

//Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ConfigurationOverride)

}

#endif
