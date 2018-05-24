/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#ifndef GAZEBOYARP_BASEPOSEVELOCITYDRIVER_H
#define GAZEBOYARP_BASEPOSEVELOCITYDRIVER_H

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
#include <yarp/os/Mutex.h>
#include <yarp/os/LockGuard.h>

#include <yarp/sig/Vector.h>

#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>


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
            
        gazebo::physics::Model* m_robot;
        gazebo::physics::LinkPtr m_baseLink;
        std::string m_baseLinkName;
        const int m_stateDimensions = 18;
        yarp::sig::Vector m_baseState;
        yarp::os::Stamp m_stamp;
        yarp::os::Mutex m_dataMutex;
        
        bool m_dataAvailable = false;
        
        gazebo::event::ConnectionPtr m_updateConnection;
               
};
#endif 
