
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
}

bool fakebotFTsensor::close()
{
    return true;
}

fakebotFTsensor::fakebotFTsensor()
{
    _useCalibration=0;
    _channels=6;
    data.resize(_channels);

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

    out.resize(data.size());

    out = data;
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
