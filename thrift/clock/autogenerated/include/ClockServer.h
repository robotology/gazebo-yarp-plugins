/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_ClockServer
#define YARP_THRIFT_GENERATOR_ClockServer

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

namespace GazeboYarpPlugins {
  class ClockServer;
}


class GazeboYarpPlugins::ClockServer : public yarp::os::Wire {
public:
  ClockServer();
  /**
   * Pause the simulation if it was running
   */
  virtual void pauseSimulation();
  /**
   * Resume the simulation if it was paused
   */
  virtual void continueSimulation();
  /**
   * Steps the simulation for the provided number of steps.
   * The input paramter is the number of steps, not the time (Usually 1 step = 1ms but this is not guaranteed)
   * @note: this function (will be) not blocking, i.e. it will return immediately. Currently calling this function
   * twice before the previous call actually ends its computation gives and undefined behavior.
   * @param numberOfSteps number of steps to simulate
   */
  virtual void stepSimulation(const std::int32_t numberOfSteps = 1);
  /**
   * Steps the simulation for the provided number of steps.
   * The input paramter is the number of steps, not the time (Usually 1 step = 1ms but this is not guaranteed)
   * @note: this function is blocking
   * @param numberOfSteps number of steps to simulate
   */
  virtual void stepSimulationAndWait(const std::int32_t numberOfSteps = 1);
  /**
   * Reset the simulation time back to zero
   */
  virtual void resetSimulationTime();
  /**
   * Get the current simulation time
   * @return the simulation time.
   */
  virtual double getSimulationTime();
  /**
   * Get the current step size in seconds.
   * @return the step size in seconds
   */
  virtual double getStepSize();
  /**
   * Reset the simulation state and time
   */
  virtual void resetSimulation();
  virtual bool read(yarp::os::ConnectionReader& connection) override;
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
