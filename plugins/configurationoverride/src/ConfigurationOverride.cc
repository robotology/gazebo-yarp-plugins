/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Lorenzo Rapetti.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include <ConfigurationOverride.hh>

#include <gazebo/physics/Model.hh>

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace std;
using namespace gazebo;

ConfigurationOverride::ConfigurationOverride()
{
}

ConfigurationOverride::~ConfigurationOverride()
{
}

void ConfigurationOverride::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    if (!_model) 
    {
        gzerr << "ConfigurationOverride plugin requires a parent.\n";
        return;
    }
    if(! _sdf->HasElement("yarpPluginConfigurationOverride") || ! _sdf->GetElement("yarpPluginConfigurationOverride")->HasAttribute("plugin_name"))
    {
        yError("ConfigurationOverride : overriden_plugin_name not found in config file\n");
        return;
    }

    sdf::ElementPtr overriden_element, new_element;
    // name of the plugin that has to be overritten
    std::string overriden_plugin_name = _sdf->GetElement("yarpPluginConfigurationOverride")->GetAttribute("plugin_name")->GetAsString();
    // sdf of the model to which the plugin is attached. the plugin to be overriden will be searched among the plugins of this model
    sdf::ElementPtr model_sdf = _sdf->GetParent();
    // search for the plugin to be overriden
    sdf::ElementPtr overriden_plugin = model_sdf->GetFirstElement();

    while(overriden_plugin)
    {
        if(overriden_plugin->HasAttribute("name") && overriden_plugin->GetAttribute("name")->GetAsString() == overriden_plugin_name)
        {
            // overriden_plugin found

            // iterate the elements of the ConfigurationOverride plugin
            new_element = _sdf->GetFirstElement();
            while(new_element)
            {
                if(new_element->GetName() != "yarpPluginConfigurationOverride")
                {
                    // if the element to be overritten is not found in the overriden_plugin, create the element
                    if(! overriden_plugin->HasElement(new_element->GetName()))
                    {
                        // Create element
                        sdf::ElementPtr created_element(new sdf::Element);
                        overriden_plugin->AddElementDescription(new_element);
                        overriden_element = overriden_plugin->GetElement(new_element->GetName());
                    }
                    else
                    {
                        // Override element
                        // write the value taken from the ConfigurationOverride element (new_element) to the overriden_plugin element (overriden_element)
                        overriden_element = overriden_plugin->GetElement(new_element->GetName());
                        overriden_element->Set<std::string>(new_element->Get<std::string>());
                    }
                    
                }
                new_element = new_element->GetNextElement();
            }
            break;
        }
        overriden_plugin = overriden_plugin->GetNextElement();
    }
    if(!overriden_plugin)
    {
        yError() <<"ConfigurationOverride : the plugin " << overriden_plugin_name << " is not found in the model " <<  model_sdf->GetAttribute("name")->GetAsString();
        return;
    }

}