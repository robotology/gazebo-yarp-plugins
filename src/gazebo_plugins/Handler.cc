/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alberto Cardellino and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "gazebo_yarp_plugins/Handler.hh"

using namespace std;
using namespace gazebo;

GazeboYarpPluginHandler* GazeboYarpPluginHandler::_handle = NULL;
yarp::os::Semaphore GazeboYarpPluginHandler::_mutex = 1;


GazeboYarpPluginHandler::GazeboYarpPluginHandler()
{
    _robotMap.clear();
    _handle = NULL;
}

GazeboYarpPluginHandler * GazeboYarpPluginHandler::getHandler()
{
    _mutex.wait();
    if (NULL == _handle)
    {
        cout << "Calling EthManager Constructor";
        _handle = new GazeboYarpPluginHandler();
        if (NULL == _handle)
            cout << "Error while calling GazeboYarpPluginHandler constructor";

    }
    _mutex.post();

    return _handle;
}

//bool GazeboYarpPluginHandler::setRobot(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
//{
//    return setRobot(get_pointer(parent), sdf);
//}

bool GazeboYarpPluginHandler::setRobot(gazebo::physics::Model* parent, sdf::ElementPtr sdf)
{
    bool ret = false;
    std::string rName;

    if(!findRobotName(sdf, &rName) )
    {
        cout << "Error, not able to find 'model' tag in the xml file" << endl;
        return false;
    }

    cout << "Inserting robot : " << rName << endl;

    if( ! _robotMap.insert(std::pair<std::string, gazebo::physics::Model*>(rName, parent) ).second)
    {
        // the element could be already present, check it out just to be sure!
        if(parent != _robotMap.at(rName) )
        {
            cout << "Error in GazeboYarpPluginHandler while inserting a new robot pointer!\n";
            cout << " The name of the robot is already present but the pointer does not match with the one already registered!!\n";
            cout << " Check config file, maybe 2 different robot was instatiated with the same name?" << endl;
            ret = false;
        }
        else
        {
            cout << "Robot already registered, pointers match. All good. This is a debug msg and can be removed." << endl;
            ret = true;
        }
    }
    else
    {
        ret = true;
        cout << "Singleton: Added a new " << rName << " robot.\n";
    }
    return ret;
}

gazebo::physics::Model* GazeboYarpPluginHandler::getRobot(std::string robotName)
{
    gazebo::physics::Model* tmp = NULL;
    cout << "Looking for robot : " << robotName << endl;
    try
    {
        tmp = _robotMap.at(robotName);
    }
    catch (...)
    {
         cout << "Robot was not found: " << robotName << endl;
         tmp = NULL;
    }
    if(NULL != tmp)
        cout << "Robot " << robotName << " was happily found!\n";
    return tmp;
}


bool GazeboYarpPluginHandler::findRobotName(sdf::ElementPtr sdf, std::string *robotName)
{
    std::cout << "()()()()()\n";
    sdf::ElementPtr model_p = sdf;

    while(model_p->GetName() != "model")
    {
        if(( model_p = model_p->GetParent() ) == NULL)
        {
            cout << "Error! model tag not found";
            return false;
        }
        sleep(1);
    }
    *robotName =  model_p->GetAttribute("name")->GetAsString();
    cout << "found model name " << *robotName << endl;
    return true;
}
