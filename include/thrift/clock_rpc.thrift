#clock module RPC server
namespace yarp gazebo

service ClockServer {
    oneway void pauseSimulation();
    oneway void continueSimulation();
    oneway void stepSimulation(1:i32 numberOfSteps = 1);
}
