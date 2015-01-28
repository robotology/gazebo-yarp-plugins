/*
 * Copyright (C) 2007-2014 Istituto Italiano di Tecnologia. RBCS Department.
 * Author: Francesco Romano
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "thrift/ClockServer.h"

namespace gazebo {
    
    class GazeboYarpClock;
    
class ClockServerImpl : public ClockServer {
public:
    
    ClockServerImpl(GazeboYarpClock& clockPlugin);
    
    virtual void pauseSimulation();
    virtual void continueSimulation();
    virtual void stepSimulation(const int32_t numberOfSteps = 1);
    virtual void stepSimulationAndWait(const int32_t numberOfSteps = 1);
    void resetSimulationTime();
    virtual double getSimulationTime();
    virtual double getStepSize();
    
private:
    
    GazeboYarpClock& m_clock;
};

}
