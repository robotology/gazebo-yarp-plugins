
// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* Copyright (C) 2012  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Alberto Cardellino
 * email: alberto.cardellino@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/// general purpose stuff.

#include <yarp/os/Time.h>
#include <stdarg.h>
#include <stdio.h>
#include <yarp/dev/PolyDriver.h>
#include <ace/config.h>
#include <ace/Log_Msg.h>

#include <string>
#include <iostream>
#include <iterator>
#include <string.h>


/// specific to this device driver.
#include <fakebotFTsensor.h>


#ifdef WIN32
#pragma warning(once:4355)
#endif

using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;

inline bool NOT_YET_IMPLEMENTED(const char *txt)
{
    return false;
}

//generic function that check is key1 is present in input bottle and that the result has size elements
// return true/false
static inline bool validate(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size)
{
    size++;  // size includes also the name of the parameter
    Bottle &tmp=input.findGroup(key1.c_str(), txt.c_str());
    if (tmp.isNull())
    {
        fprintf(stderr, "%s not found\n", key1.c_str());
        return false;
    }

    if( tmp.size() != size)
    {
        fprintf(stderr, "%s incorrect number of entries\n", key1.c_str());
        return false;
    }

    out=tmp;

    return true;
}

bool fakebotFTsensor::fromConfig(yarp::os::Searchable &_config)
{
    return true;
};

bool fakebotFTsensor::close()
{
    return true;
}

fakebotFTsensor::fakebotFTsensor()
{
    _useCalibration=0;
    _channels=6;

    status=IAnalogSensor::AS_OK;
}

fakebotFTsensor::~fakebotFTsensor()
{

}

bool fakebotFTsensor::open(yarp::os::Searchable &config)
{
    Property prop;
    std::string str=config.toString().c_str();

    if(!fromConfig(config))
        return false;

    prop.fromString(str.c_str());


    // TODO fix this!
#warning "<><> TODO: This is a copy of the mcs map. Verify that things will never change after this copy or use a pointer (better) <><>"
    return true;
}


/*! Read a vector from the sensor.
 * @param out a vector containing the sensor's last readings.
 * @return AS_OK or return code. AS_TIMEOUT if the sensor timed-out.
 **/
int fakebotFTsensor::read(yarp::sig::Vector &out)
{
    // This method gives data to the analogServer

    mutex.wait();
    status = AS_OK;

    // TODO

    // Che TIPO di dato è (int, double...)??? Chi lo decide???
    out.resize(data.size() * _channels);

    std::map<unsigned int,std::string>::iterator iter;
    int idx = 0;
    for(iter = this->bMap.begin(); iter!=this->bMap.end(); iter++)
    {
        yarp::sig::Vector iterVec = data[iter->second];
        out[idx*_channels + 0] = iterVec[0];
        out[idx*_channels + 1] = iterVec[1];
        out[idx*_channels + 2] = iterVec[2];
        out[idx*_channels + 3] = iterVec[3];
        out[idx*_channels + 4] = iterVec[4];
        out[idx*_channels + 5] = iterVec[5];
        idx++;
    }
    mutex.post();

    return status;
}

int fakebotFTsensor::getState(int ch)
{
    printf("getstate\n");
    return AS_OK;
}

int fakebotFTsensor::getChannels()
{
//     McBoard *joint_p = getFTpointer(j);  // TODO su quale sensore?
//     int8_t channels;
//     if( joint_p->getItem(GET_ANALOG_INPUTS,   NULL, 0, REPLY_ANALOG_INPUTS, &channels, sizeof(channels)) )
//         yError() << "Error while getting number of channels";
//     int ret = (int) channels;
//     return ret;
//    return 0;

     return _channels;
}

int fakebotFTsensor::calibrateSensor()
{
    return AS_OK;
}

int fakebotFTsensor::calibrateSensor(const yarp::sig::Vector& value)
{
    return AS_OK;
}

int fakebotFTsensor::calibrateChannel(int ch)
{
    return AS_OK;
}

int fakebotFTsensor::calibrateChannel(int ch, double v)
{
    return AS_OK;
}
