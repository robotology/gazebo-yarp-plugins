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
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>

#include <string>
#include <vector>

using namespace gazebo;

namespace GazeboYarpPlugins {

/**
 * Split a string in a vector of string given a (single char) delimiter
 */
std::vector<std::string> splitString(const std::string &s, const std::string &delim)
{
    std::vector<std::string> retVec;

    size_t start = s.find_first_not_of(delim), end=start;

    while (start != std::string::npos)
    {
        // Find next occurence of delimiter
        end = s.find(delim, start);
        // Push back the token found into vector
        retVec.push_back(s.substr(start, end-start));
        // Skip all occurences of the delimiter to find new start
        start = s.find_first_not_of(delim, end);
    }

    return retVec;
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
            yError() << "GazeboYarpPlugins error: failure in loading configuration for model" << _model->GetName() << "\n"
                      << "GazeboYarpPlugins error: yarpConfigurationFile : " << ini_file_name << "\n"
                      << "GazeboYarpPlugins error: yarpConfigurationFile absolute path : " << ini_file_path;
            return false;
        }
    }
    return true;
}

bool addGazeboEnviromentalVariablesSensor(gazebo::sensors::SensorPtr _sensor,
                                         sdf::ElementPtr _sdf,
                                         yarp::os::Property & plugin_parameters)
{
    // Prefill the property object with some gazebo-yarp-plugins "Enviromental Variables"
    // (not using the env variable in fromConfigFile(const ConstString& fname, Searchable& env, bool wipe)
    // method because we want the variable defined here to be overwritable by the user configuration file
#if GAZEBO_MAJOR_VERSION >= 7
    std::string gazeboYarpPluginsSensorName = _sensor->Name();
#else
    std::string gazeboYarpPluginsSensorName = _sensor->GetName();
#endif
    plugin_parameters.put("gazeboYarpPluginsSensorName",gazeboYarpPluginsSensorName.c_str());

    // Extract the robot name from the sensor scoped name
#if GAZEBO_MAJOR_VERSION >= 7
    std::string scopedSensorName = _sensor->ScopedName();
#else
    std::string scopedSensorName = _sensor->GetScopedName();
#endif

    std::vector<std::string> explodedScopedSensorName = splitString(scopedSensorName,":");

    // The vector should be at least of 3 elements because
    // scopedSensorName should be something similar to
    // worldName::modelName::linkOrJointName::sensorName
    if( explodedScopedSensorName.size() < 3 )
    {
        yError() << "GazeboYarpPlugins warning: unexpected scopedSensorName " << scopedSensorName;
        yError() << "GazeboYarpPlugins warning: gazeboYarpPluginsRobotName not set in sensor " << gazeboYarpPluginsSensorName;
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
        if (ini_file_path != "" && plugin_parameters.fromConfigFile(ini_file_path.c_str(),wipe))
        {
            return true;
        }
        else
        {
#if GAZEBO_MAJOR_VERSION >= 7
    std::string sensorName = _sensor->Name();
#else
    std::string sensorName = _sensor->GetName();
#endif
            yError()  << "GazeboYarpPlugins error: failure in loading configuration for sensor " << sensorName << "\n"
                      << "GazeboYarpPlugins error: yarpConfigurationFile : " << ini_file_name << "\n"
                      << "GazeboYarpPlugins error: yarpConfigurationFile absolute path : " << ini_file_path ;
            return false;
        }
    }
    return true;
}


}
