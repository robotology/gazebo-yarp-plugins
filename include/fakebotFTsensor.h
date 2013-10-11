// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_hardware_modules 
 * \defgroup analogSensorEth
 *
 * To Do: add description
 *
 */

#ifndef __fakebotFTsensor_h__
#define __fakebotFTsensor_h__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>

#include <iCub/DebugInterfaces.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>
#include <string>
#include <list>
#include <map>

#include <boost/concept_check.hpp>

#include <iCub/FactoryInterface.h>
#include <iCub/LoggerInterfaces.h>

namespace yarp{
    namespace dev{
        class fakebotFTsensor;
    }
}

// class AnalogData
// {
// private:
//     double *_data;
//     int _size;
//     int _bufferSize;
// public:
//     AnalogData(int ch, int buffsize): _data(0), _size(ch), _bufferSize(buffsize)
//     {
//         _data=new double[_bufferSize];
//         for(int k=0;k<_bufferSize;k++)
//             _data[k]=0;
//     }
//     ~AnalogData()
//     {
//         delete [] _data;
//     }
// 
//     inline double &operator[](int i)
//     { return _data[i]; }
// 
//     inline int size() 
//     { return _size; }
// 
//     inline double *getBuffer()
//     {return _data;}
// };



typedef int AnalogDataFormat;
/*! class yarp::dev::fakebotFTsensor
 *
 */
class yarp::dev::fakebotFTsensor:     public yarp::dev::DeviceDriver,
                                      public yarp::dev::IAnalogSensor
{
private:

////////////////////
    // parameters
    int             _channels;
    short           _useCalibration;

//     AnalogData *data;
    short status;

    double timeStamp;
    yarp::os::Semaphore mutex;

    yarp::os::Bottle initMsg;
    yarp::os::Bottle speedMsg;
    yarp::os::Bottle closeMsg;
    std::string deviceIdentifier;

    // Read useful data from config and check fir correctness
    bool fromConfig(yarp::os::Searchable &config);

public:

    fakebotFTsensor();
    ~fakebotFTsensor();

    bool open(yarp::os::Searchable &config);
    bool close();

    //IAnalogSensor interface
    virtual int read(yarp::sig::Vector &out);
    virtual int getState(int ch);
    virtual int getChannels();
    virtual int calibrateChannel(int ch, double v);
    virtual int calibrateSensor();
    virtual int calibrateSensor(const yarp::sig::Vector& value);

    virtual int calibrateChannel(int ch);

    std::map<std::string, yarp::sig::Vector> data;
    std::map<unsigned int, std::string> bMap;

    void setDeviceId(std::string id)
    {
        deviceIdentifier=id;
    }

    std::string getDeviceId()
    {
        return deviceIdentifier;
    }
};


#endif   // __fakebotFTsensor_h__


