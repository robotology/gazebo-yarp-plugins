#clock module RPC server
namespace yarp gazebo

service ClockServer {
    /** Pause the simulation if it was running
     */
    oneway void pauseSimulation();
    
    /** Resume the simulation if it was paused
     */
    oneway void continueSimulation();
    
    /**
     * Steps the simulation for the provided number of steps. 
     * The input paramter is the number of steps, not the time (Usually 1 step = 1ms but this is not guaranteed)
     * @note: this function (will be) not blocking, i.e. it will return immediately
     * @param numberOfSteps number of steps to simulate
     */
    oneway void stepSimulation(1:i32 numberOfSteps = 1);
    
    /**
     * Steps the simulation for the provided number of steps. 
     * The input paramter is the number of steps, not the time (Usually 1 step = 1ms but this is not guaranteed)
     * @note: this function is blocking
     * @param numberOfSteps number of steps to simulate
     */
    void stepSimulationAndWait(1:i32 numberOfSteps = 1);
    
    /** Returns the simulation time.
     */
    double getSimulationTime();
    
}
