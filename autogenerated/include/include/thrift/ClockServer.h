// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_ClockServer
#define YARP_THRIFT_GENERATOR_ClockServer

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

namespace gazebo {
  class ClockServer;
}


class gazebo::ClockServer : public yarp::os::Wire {
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
   * @note: this function (will be) not blocking, i.e. it will return immediately
   * @param numberOfSteps number of steps to simulate
   */
  virtual void stepSimulation(const int32_t numberOfSteps = 1);
  /**
   * Steps the simulation for the provided number of steps.
   * The input paramter is the number of steps, not the time (Usually 1 step = 1ms but this is not guaranteed)
   * @note: this function is blocking
   * @param numberOfSteps number of steps to simulate
   */
  virtual void stepSimulationAndWait(const int32_t numberOfSteps = 1);
  /**
   * Returns the simulation time.
   */
  virtual double getSimulationTime();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif

