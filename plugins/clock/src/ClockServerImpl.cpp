/*
 * Copyright (C) 2007-2014 Istituto Italiano di Tecnologia. RBCS Department.
 * Author: Francesco Romano
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "ClockServerImpl.h"
#include "Clock.hh"
#include <iostream>

namespace GazeboYarpPlugins {
    
    ClockServerImpl::ClockServerImpl(gazebo::GazeboYarpClock& clockPlugin)
    : m_clock(clockPlugin) {}
    
    void ClockServerImpl::pauseSimulation()
    {
        m_clock.clockPause();
    }
    
    void ClockServerImpl::continueSimulation()
    {
        m_clock.clockContinue();
    }

    void ClockServerImpl::stepSimulation(const int32_t numberOfSteps)
    {
        m_clock.clockStep(static_cast<unsigned>(numberOfSteps));
    }
    
    void ClockServerImpl::stepSimulationAndWait(const int32_t numberOfSteps)
    {
        m_clock.clockStep(static_cast<unsigned>(numberOfSteps));
    }
    
    void ClockServerImpl::resetSimulationTime()
    {
        m_clock.resetSimulationTime();
    }

    void ClockServerImpl::resetSimulation()
    {
        m_clock.resetSimulation();
    }
    
    double ClockServerImpl::getSimulationTime()
    {
        return m_clock.getSimulationTime().Double();
    }
    
    double ClockServerImpl::getStepSize()
    {
        return m_clock.getStepSize();
    }
}
