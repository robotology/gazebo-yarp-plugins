/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia iCub Facility & ADVR
 * Authors: Lorenzo Natale and Paul Fitzpatrick and Enrico Mingo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <coman.h>

#include <yarp/sig/all.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/os/all.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::sig::file;
using namespace yarp::dev;

bool coman::open(yarp::os::Searchable& config) 
{
  //if there is a .ini file, directly load it avoiding copyng all the data from config
  if( config.check("gazebo_ini_file_path") ) {
      plugin_parameters.fromConfigFile(config.find("gazebo_ini_file_path").asString().c_str()); 
  }
  
  setPIDs();
  
  return true;
}


/*
bool coman::threadInit()
{
    printf("FakeBot thread started\n");
    return true;
}

void coman::threadRelease()
{
    printf("FakeBot thread closed\n");
}

void coman::run()
{
//    static int count=0;
//    count++;
//    if (count%100==0)
//    {
//        printf("======\nfakebot thread looped [%d] times\n", count);
//        printf("Current position: %s\n", pos.toString().c_str());
//        printf("Current reference position: %s\n", ref_pos.toString().c_str());
//        printf("Reference speed: %s\n", ref_speed.toString().c_str());
//    }

    for(int j=0; j<_robot_number_of_joints; ++j)
    {
        // handle position mode
        if (control_mode[j]==VOCAB_CM_POSITION)
        {
            if ( (pos[j]-ref_pos[j]) < -ROBOT_POSITION_TOLERANCE)
            {
                pos[j]=pos[j]+ref_speed[j]*RateThread::getRate()/1000.0;
                motion_done[j]=false;
            }
            else if ( (pos[j]-ref_pos[j]) >ROBOT_POSITION_TOLERANCE)
            {
                pos[j]=pos[j]-ref_speed[j]*RateThread::getRate()/1000.0;
                motion_done[j]=false;
            }
            else
                motion_done[j]=true;
        }

        pos[j]=pos[j]+Random::normal(0,0.01);
    }
}
*/