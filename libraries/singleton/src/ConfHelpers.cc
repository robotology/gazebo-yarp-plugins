/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alberto Cardellino and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "ConfHelpers.hh"

#include <yarp/dev/PolyDriver.h>

#include <gazebo/physics/Entity.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>

#include <yarp/os/Property.h>

#include <string>
#include <vector>

using namespace gazebo;

namespace GazeboYarpPlugins {

/**
 * Split a string in a vector of string given a delimiter
 */
std::vector<std::string> splitString(const std::string &s, const std::string &delims)
{
    std::vector<std::string> result;
    std::string::size_type pos = 0;
    while (std::string::npos != (pos = s.find_first_not_of(delims, pos))) {
        auto pos2 = s.find_first_of(delims, pos);
        result.emplace_back(s.substr(pos, std::string::npos == pos2 ? pos2 : pos2 - pos));
        pos = pos2;
    }
    return result;
}

bool addGazeboEnviromentalVariablesModel(gazebo::physics::ModelPtr _parent,
                                         sdf::ElementPtr _sdf,
                                         yarp::os::Property & plugin_parameters)
{
    // Prefill the property object with some gazebo-yarp-plugins "Enviromental Variables"
    // (not using the env variable in fromConfigFile(const ConstString& fname, Searchable& env, bool wipe)
    // method because we want the variable defined here to be overwritable by the user configuration file
    std::string gazeboYarpPluginsRobotName = _parent->GetName();
    plugin_parameters.put("gazeboYarpPluginsRobotName",gazeboYarpPluginsRobotName.c_str());
    return true;
}

bool loadConfigModelPlugin(physics::ModelPtr _model,
                           sdf::ElementPtr _sdf,
                           yarp::os::Property& plugin_parameters)
{
    if (_sdf->HasElement("yarpConfigurationFile")) {
        std::string ini_file_name = _sdf->Get<std::string>("yarpConfigurationFile");
        std::string ini_file_path = gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);

        GazeboYarpPlugins::addGazeboEnviromentalVariablesModel(_model,_sdf,plugin_parameters);

        bool wipe = false;
        if (ini_file_path != "" && plugin_parameters.fromConfigFile(ini_file_path.c_str(),wipe)) {
            return true;
        } else {
            std::cerr << "GazeboYarpPlugins error: failure in loading configuration for model " << _model->GetName() << std::endl
                      << "GazeboYarpPlugins error: yarpConfigurationFile : " << ini_file_name << std::endl
                      << "GazeboYarpPlugins error: yarpConfigurationFile absolute path : " << ini_file_path << std::endl;
            return false;
        }
    }
}

bool addGazeboEnviromentalVariablesSensor(gazebo::sensors::SensorPtr _sensor,
                                         sdf::ElementPtr _sdf,
                                         yarp::os::Property & plugin_parameters)
{
    // Prefill the property object with some gazebo-yarp-plugins "Enviromental Variables"
    // (not using the env variable in fromConfigFile(const ConstString& fname, Searchable& env, bool wipe)
    // method because we want the variable defined here to be overwritable by the user configuration file
    std::string gazeboYarpPluginsSensorName = _sensor->GetName();
    plugin_parameters.put("gazeboYarpPluginsSensorName",gazeboYarpPluginsSensorName.c_str());

    // Extract the robot name from the sensor scoped name
    std::string scopedSensorName = _sensor->GetScopedName();
    std::vector<std::string> explodedScopedSensorName = splitString(scopedSensorName,"::");

    // The vector should be at least of 3 elements because
    // scopedSensorName should be something similar to
    // worldName::modelName::linkOrJointName::sensorName
    if( explodedScopedSensorName.size() < 3 )
    {
        std::cerr << "GazeboYarpPlugins warning: unexpected scopedSensorName " << scopedSensorName << std::endl;
        std::cerr << "GazeboYarpPlugins warning: gazeboYarpPluginsRobotName not set in sensor " << gazeboYarpPluginsSensorName << std::endl;
        return false;
    }

    std::string gazeboYarpPluginsRobotName = explodedScopedSensorName[explodedScopedSensorName.size()-3];
    plugin_parameters.put("gazeboYarpPluginsRobotName",gazeboYarpPluginsRobotName.c_str());
    return true;
}

bool loadConfigSensorPlugin(sensors::SensorPtr _sensor,
                            sdf::ElementPtr _sdf,
                            yarp::os::Property& plugin_parameters)
{
    if (_sdf->HasElement("yarpConfigurationFile")) {
        std::string ini_file_name = _sdf->Get<std::string>("yarpConfigurationFile");
        std::string ini_file_path = gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);

        GazeboYarpPlugins::addGazeboEnviromentalVariablesSensor(_sensor,_sdf,plugin_parameters);

        bool wipe = false;
        if (ini_file_path != "" && plugin_parameters.fromConfigFile(ini_file_path.c_str(),wipe)) {
            return true;
        } else {
            std::cerr << "GazeboYarpPlugins error: failure in loading configuration for sensor " << _sensor->GetName() << std::endl
                      << "GazeboYarpPlugins error: yarpConfigurationFile : " << ini_file_name << std::endl
                      << "GazeboYarpPlugins error: yarpConfigurationFile absolute path : " << ini_file_path << std::endl;
            return false;
        }
    }
}


}
