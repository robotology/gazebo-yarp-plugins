// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_ObjectsServer
#define YARP_THRIFT_GENERATOR_ObjectsServer

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

namespace gazebo {
  class ObjectsServer;
}


class gazebo::ObjectsServer : public yarp::os::Wire {
public:
  ObjectsServer() { yarp().setOwner(*this); }
  /**
   * Get attach an object to a link of the robot.
   * @param link_name Name of the link
   * @param object_name Name of the box to be operated
   * @param width width of the box to be operated
   * @param height height of the box to be operated
   * @param length length of the box to be operated
   * @return true if success, false otherwise
   */
  virtual bool attach(const std::string& link_name, const std::string& object_name, const double width, const double height, const double length);
  /**
   * Detaches an object to a link of the robot.
   * @param object_name Name of the object
   * @return true if success, false otherwise
   */
  virtual bool detach(const std::string& object_name);
  /**
   * Delete an object from Gazebo environment.
   * @param object_name Name of the object
   * @return true if success, false otherwise
   */
  virtual bool deleteObject(const std::string& object_name);
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif

