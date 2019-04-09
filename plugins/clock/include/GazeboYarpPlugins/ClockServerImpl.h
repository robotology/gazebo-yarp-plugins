/*
 * Copyright (C) 2007-2015 Istituto Italiano di Tecnologia. RBCS Department.
 * Author: Francesco Romano
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include <ClockServer.h>

namespace GazeboYarpPlugins {
    class ClockServerImpl;
}

namespace gazebo {
    class GazeboYarpClock;
}

class GazeboYarpPlugins::ClockServerImpl : public GazeboYarpPlugins::ClockServer {
public:
    
    ClockServerImpl(gazebo::GazeboYarpClock& clockPlugin);
    
    virtual void pauseSimulation();
    virtual void continueSimulation();
    virtual void stepSimulation(const int32_t numberOfSteps = 1);
    virtual void stepSimulationAndWait(const int32_t numberOfSteps = 1);
    virtual void resetSimulationTime();
    virtual void resetSimulation();
    virtual double getSimulationTime();
    virtual double getStepSize();
    
private:
    
    gazebo::GazeboYarpClock& m_clock;
};
