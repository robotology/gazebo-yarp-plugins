/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "ControlBoardDriver.h"
#include <yarp/os/LogStream.h>

namespace yarp {
    namespace dev {
      

        VAS_status GazeboYarpControlBoardDriver::getVirtualAnalogSensorStatus(int ch)
        {
            if (ch < 0 || ch >= this->getVirtualAnalogSensorChannels())
            {
                yError() << "GazeboYarpControlBoardDriver: VirtualAnalogServer: getState failed: requested channel " << ch << 
                            "while the client is configured wtih " << this->getVirtualAnalogSensorChannels() << "channels";
            
                return VAS_status::VAS_ERROR;
            }
                   
            return VAS_status::VAS_OK;
        }
        
        int GazeboYarpControlBoardDriver::getVirtualAnalogSensorChannels()
        {
            return m_numberOfJoints;
        }
        
        bool GazeboYarpControlBoardDriver::updateVirtualAnalogSensorMeasure(yarp::sig::Vector &measure)
        {
            if (m_useVirtualAnalogSensor)
            {
                if (measure.size() != m_numberOfJoints)
                {
                    yError() << "GazeboYarpControlBoardDriver: VirtualAnalogServer: assert size error : Vector lengths do not match";
                    return false;
                }
                
                for (size_t j = 0; j < m_numberOfJoints; j++)
                {
                    this->updateVirtualAnalogSensorMeasure(j, measure[j]);
                }
                return true;
            }
            return false;
        }
        
        bool GazeboYarpControlBoardDriver::updateVirtualAnalogSensorMeasure(int ch, double &measure)
        {
            if (m_useVirtualAnalogSensor)
            {
                if (ch < 0 || ch >= this->getVirtualAnalogSensorChannels())
                {
                    yError() << "GazeboYarpControlBoardDriver: VirtualAnalogServer: index out of bounds";
                    return false;
                }
                
                m_measTorques[ch] = measure;
                return true;             
            }
            return false;
        }
      
    }
}
