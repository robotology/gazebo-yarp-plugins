// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_WorldInterfaceServer
#define YARP_THRIFT_GENERATOR_WorldInterfaceServer

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <Color.h>
#include <Pose.h>

namespace GazeboYarpPlugins {
  class WorldInterfaceServer;
}


class GazeboYarpPlugins::WorldInterfaceServer : public yarp::os::Wire {
public:
  WorldInterfaceServer();
  /**
   * Make a shpere.
   * @param radius radius of the sphere [m]
   * @param pose pose of the sphere [m]
   * @param color color of the sphere
   * @return returns a string that contains the name of the object in the world
   */
  virtual std::string makeSphere(const double radius, const Pose& pose, const Color& color);
  /**
   * Make a shpere.
   * @param width box width [m]
   * @param height box height[m]
   * @param thickness box thickness [m]
   * @param pose pose of the box [m]
   * @param color color of the box
   * @return returns a string that contains the name of the object in the world
   */
  virtual std::string makeBox(const double width, const double height, const double thickness, const Pose& pose, const Color& color);
  /**
   * Make a cylinder.
   * @param radius radius of the cylinder [m]
   * @param length lenght of the cylinder [m]
   * @param pose pose of the cylinder [m]
   * @param color color of the cylinder
   * @return returns a string that contains the name of the object in the world
   */
  virtual std::string makeCylinder(const double radius, const double length, const Pose& pose, const Color& color);
  /**
   * Set new object pose.
   * @param id object id
   * @param pose new pose
   * @return returns true or false on success failure
   */
  virtual bool setPose(const std::string& id, const Pose& pose);
  /**
   * Enable/disables gravity for an object
   * @param id object id
   * @param enable 1 to enable gravity, 0 otherwise
   * @return returns true or false on success failure
   */
  virtual bool enableGravity(const std::string& id, const bool enable);
  /**
   * Enable/disables collision detection for an object
   * @param id object id
   * @param enable 1 to enable collision detection, 0 otherwise
   * @return returns true or false on success failure
   */
  virtual bool enableCollision(const std::string& id, const bool enable);
  /**
   * Get object pose.
   * @param id string that identifies object in gazebo (returned after creation)
   * @return returns value of the pose
   */
  virtual Pose getPose(const std::string& id);
  /**
   * Load a model from file.
   * @param id string that specifies the name of the model
   * @return returns true/false on success failure.
   */
  virtual bool loadModelFromFile(const std::string& filename);
  /**
   * Delete all objects in the world.
   */
  virtual bool deleteAll();
  /**
   * List id of all objects that have been added to the world.
   * @return return a list of string containing the id of the objects
   */
  virtual std::vector<std::string>  getList();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
