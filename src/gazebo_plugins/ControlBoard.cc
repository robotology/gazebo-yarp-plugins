/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */
#include <gazebo_yarp_plugins/ControlBoard.hh>
#include "gazebo_yarp_plugins/Handler.hh"

using namespace std;
namespace gazebo
{
    
GZ_REGISTER_MODEL_PLUGIN(GazeboYarpControlBoard)

    GazeboYarpControlBoard::GazeboYarpControlBoard() : _yarp()
    {

    }

    void GazeboYarpControlBoard::Init()
    {
        std::cout<<"*** COMAN GAZEBO YARP PLUGIN ***"<<std::endl;
        if (!_yarp.checkNetwork())
            std::cout<<"Sorry YARP network does not seem to be available, is the yarp server available?"<<std::endl;
        else
            std::cout<<"YARP Server found!"<<std::endl;
    }

    GazeboYarpControlBoard::~GazeboYarpControlBoard()
    {
        _wrapper.close();
        std::cout<<"Goodbye!"<<std::endl;
    }

    /**
     * Saves the gazebo pointer, creates the device driver
     */
    void GazeboYarpControlBoard::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        GazeboYarpPluginHandler::getHandler()->setRobot(get_pointer(_parent), _sdf);

        this->_robot = _parent;

        // Add my gazebo device driver to the factory.
        yarp::dev::Drivers::factory().add(new yarp::dev::DriverCreatorOf<yarp::dev::GazeboYarpControlBoardDriver>
                                          ("gazebo_controlboard", "controlboardwrapper2", "GazeboYarpControlBoardDriver"));

        std::cout<<"\n Initting Wrapper\n"<<std::endl;
        //Getting .ini configuration file from sdf
        bool configuration_loaded = false;



        yarp::os::Bottle wrapper_group;
        yarp::os::Bottle driver_group;
        if(_sdf->HasElement("yarpConfigurationFile") )
        {
            std::string ini_file_name = _sdf->Get<std::string>("yarpConfigurationFile");
            std::string ini_file_path = gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);

            if( ini_file_path != "" && _parameters.fromConfigFile(ini_file_path.c_str()) )
            {
                std::cout << "Found yarpConfigurationFile: loading from " << ini_file_path << std::endl; 
                _parameters.put("gazebo_ini_file_path",ini_file_path.c_str());
            
//                std::cout << "<<<<<< Just read file\n " << _parameters.toString() << "\n>>>>>>\n";
                wrapper_group = _parameters.findGroup("WRAPPER");
                if(wrapper_group.isNull())
                {
                    printf("GazeboYarpControlBoard::Load  Error: [WRAPPER] group not found in config file\n");
                    return;
                }

                configuration_loaded = true;
            }
            
        }
        if( !configuration_loaded )
        {
            std::cout << "File .ini not found, quitting\n" << std::endl;
            return;
        }

        //Now I love everything and every interface
        std::ostringstream archive_stream;
        boost::archive::text_oarchive archive(archive_stream);
        uintptr_t cast_boost_to_pointer=(uintptr_t)_parent.get();
        archive<<cast_boost_to_pointer;
        _parameters.put("loving_gazebo_pointer",archive_stream.str().c_str());

        _wrapper.open(wrapper_group);
    
        if (!_wrapper.isValid())
            fprintf(stderr, "wrapper did not open\n");
        else
            fprintf(stderr, "wrapper opened correctly\n");

        if( !_wrapper.view(_iWrap) )
        {
            printf("Wrapper interface not found\n");
        }

        yarp::os::Bottle *netList = wrapper_group.find("networks").asList();
        if(netList->isNull())
        {
            printf("ERROR, net list to attach to was not found, exiting\n");
            _wrapper.close();
            _controlBoard.close();
            return;
        }

        yarp::dev::PolyDriverList p;

        for(int n=0; n<netList->size(); n++)
        {
            yarp::os::ConstString driverName( netList->get(n).asString().c_str());


            driver_group = _parameters.findGroup(driverName.c_str());
            if(driver_group.isNull())
            {
                printf("GazeboYarpControlBoard::Load  Error: [%s] group not found in config file\n", driverName.c_str());
                return;
            }

            yarp::os::Property driver_property(driver_group.toString().c_str());
            driver_property.put("loving_gazebo_pointer",archive_stream.str().c_str());
            driver_property.put("name", driverName.c_str());
//            std::cout << "before open: params are " << driver_property.toString() << std::endl;

            _parameters.put("loving_gazebo_pointer",archive_stream.str().c_str());
            _parameters.put("name", driverName.c_str());
            _parameters.fromString(driver_group.toString(), false);
//            std::cout << "before open: params are " << _parameters.toString() << std::endl;

//            _controlBoard.open(driver_property);
            _controlBoard.open(_parameters);

            if (!_controlBoard.isValid())
                fprintf(stderr, "controlBoard did not open\n");
            else
                printf("controlBoard opened correctly\n");

            p.push(&_controlBoard, netList->get(n).asString().c_str());
        }


        if(!_iWrap->attachAll(p))
        {
            printf("Error while attaching wrapper to device\n");
            _wrapper.close();
            _controlBoard.close();
            return;
        }

        if(_sdf->HasElement("initialConfiguration") )
        {
            std::stringstream configuration_ss(_sdf->Get<std::string>("initialConfiguration"));

            double number_of_dofs = _parameters.findGroup("WRAPPER").find("joints").asDouble();
            yarp::os::Bottle *chain_name = _parameters.findGroup("WRAPPER").find("networks").asList();
            yarp::os::Bottle joint_names = _parameters.findGroup(chain_name->toString()).tail();

            double tmp = 0.0;
            yarp::sig::Vector initial_config(number_of_dofs);
            unsigned int counter = 1;
            while(configuration_ss>>tmp)
            {
                if(counter > number_of_dofs)
                {
                    std::cout<<"To many element in initial configuration, stopping at element "<<counter<<std::endl;
                    break;
                }
                initial_config[counter-1] = tmp;
                counter++;
            }
            std::cout<<"INITIAL CONFIGURATION IS: "<<initial_config.toString()<<std::endl;

            for(unsigned int i = 0; i < number_of_dofs; ++i)
            {
                gazebo::math::Angle a;
                a.SetFromRadian(initial_config[i]);
                std::string joint_name = "COMAN::"+joint_names.findGroup("jointNames").get(i+1).toString();
                _robot->GetJoint(joint_name)->SetAngle(0,a);
            }
            sleep(3);
            printf("Device initialized correctly,...........");
        }

        printf("Device initialized correctly, now sitting and waiting cause I am just the main of the yarp device, and the coman is linked to the onUpdate event of gazebo\n");
        std::cout<<"Loaded GazeboYarpControlBoard Plugin"<<std::endl;
    }


}
