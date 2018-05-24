/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#ifndef GAZEBOYARP_BASEPOSEVELOCITY_H
#define GAZEBOYARP_BASEPOSEVELOCITY_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>

#include <gazebo/physics/Link.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Matrix3.hh>
#include <ignition/math.hh>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PreciselyTimed.h>

#include <yarp/os/Stamp.h>
#include <yarp/sig/Matrix.h>


namespace yarp
{
    namespace dev
    {
        class GazeboYarpBasePoseVelocityDriver;
    }
}
class yarp::dev::GazeboYarpBasePoseVelocityDriver : public yarp::dev::IAnalogSensor,
                                                    public yarp::dev::DeviceDriver,
                                                    public yarp::dev::IPreciselyTimed
    {
        public:
        GazeboYarpBasePoseVelocityDriver();
        virtual ~GazeboYarpBasePoseVelocityDriver();
        
        
        // Device Driver
        virtual bool open(yarp::os::Searchable& config);
        virtual bool close();
    
        // Analog sensor
        virtual int read(yarp::sig::Vector &out);
        
        // Analog sensor unimplemented - not required in simulation
        virtual int getState(int ch);
        virtual int getChannels();
        virtual int calibrateChannel(int ch);
        virtual int calibrateChannel(int ch, double v);
        virtual int calibrateSensor();
        virtual int calibrateSensor(const yarp::sig::Vector& value);
    
        // Precisely Timed
        virtual yarp::os::Stamp getLastInputStamp(); 
    
        void onUpdate(const gazebo::common::UpdateInfo& /*_info*/);
    
        private:
               
    };
#endif 