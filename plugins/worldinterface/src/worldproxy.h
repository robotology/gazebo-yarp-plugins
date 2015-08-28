#ifndef __WORLDPROXY__
#define __WORLDPROXY__

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include "WorldInterfaceServer.h"
#include <yarp/os/Mutex.h>
#include <yarp/os/Semaphore.h>

#include <map>
#include <queue>


class SynchronizationHelper
{
  yarp::os::Mutex mutex;
  yarp::os::Semaphore semaphore;
  int queued;
public:
  SynchronizationHelper():
   semaphore(0),
   queued(0)
   {}
  
  void wait()
  {
    mutex.lock();
    queued++;
    mutex.unlock();
    semaphore.wait();
  }
 
  void signalAll()
  {
    int toBePosted;
    mutex.lock();
    toBePosted=queued;
    queued=0;
    mutex.unlock();
    while(toBePosted--)
    {
       semaphore.post();      
    }
  }
};


class WorldProxy:public GazeboYarpPlugins::WorldInterfaceServer
{
  yarp::os::Mutex mutex;
  SynchronizationHelper synchHelper;
  bool isSynchro;
  
  struct ObjectsList: public std::map<std::string, gazebo::physics::ModelPtr> 
  {
    int count; //* count number of objects inserted in the world //
    
    ObjectsList(): count(0)
    {}
    
  };
  
  typedef ObjectsList::iterator ObjectsListIt;
  typedef ObjectsList::const_iterator ObjectsListConstIt;
  
  struct PoseCmd
  {
    //gazebo::physics::ModelPtr model;
    std::string name;
    gazebo::math::Pose pose;
  };
  
  class PositionCmdList: public std::queue<PoseCmd> 
  {};
  
  gazebo::physics::WorldPtr world;
  gazebo::physics::ModelPtr model;
  ObjectsList objects;
  
  PositionCmdList posecommands; 
  
public:
  WorldProxy();
  ~WorldProxy();
  
 /**
   * Make a shpere.
   * @param radius radius of the sphere [m]
   * @param pose pose of the sphere [m]
   * @param color color of the sphere
   * @return returns a string that contains the name of the object in the world
   */
  virtual std::string makeSphere(const double radius, const GazeboYarpPlugins::Pose& pose, const GazeboYarpPlugins::Color& color);
  /**
   * Make a shpere.
   * @param width box width [m]
   * @param height box height[m]
   * @param thickness box thickness [m]
   * @param pose pose of the box [m]
   * @param color color of the box
   * @return returns a string that contains the name of the object in the world
   */
  virtual std::string makeBox(const double width, const double height, const double thickness, const GazeboYarpPlugins::Pose& pose, const GazeboYarpPlugins::Color& color);
  /**
   * Make a cylinder.
   * @param radius radius of the cylinder [m]
   * @param length lenght of the cylinder [m]
   * @param pose pose of the cylinder [m]
   * @param color color of the cylinder
   * @return returns a string that contains the name of the object in the world
   */
  virtual std::string makeCylinder(const double radius, const double length, const GazeboYarpPlugins::Pose& pose, const GazeboYarpPlugins::Color& color);
  /**
   * Set new object pose.
   * @param id object id
   * @param pose new pose
   * @return returns true or false on success failure
   */
  virtual bool setPose(const std::string& id, const GazeboYarpPlugins::Pose& pose);
  /**
   * Get object pose.
   * @param id string that identifies object in gazebo (returned after creation)
   * @return returns value of the pose
   */
  virtual GazeboYarpPlugins::Pose getPose(const std::string& id);
  /**
   * Delete all objects in the world.
   */
  virtual bool deleteAll();
  /**
   * List id of all objects that have been added to the world.
   * @return return a list of string containing the id of the objects
   */
  virtual std::vector<std::string>  getList();

  /**
   * Load a model from file.
   * @param id string that specifies the name of the model
   * @return returns true/false on success failure.
   */
  virtual bool loadModelFromFile(const std::string& filename);

  
  void attachWorldPointer(gazebo::physics::WorldPtr p)
  {
    world=p;
  }
  
  void attachModelPointer(gazebo::physics::ModelPtr p)
  {
    model=p;
  }
  
  void update(const gazebo::common::UpdateInfo &);
  
  /**
   * Wait for engine to perform update step
   */
  void waitForEngine();
  
  /**
   * Signal engine has performed one step. Use internally within update
   */
  void signalEngine();
  
  /**
   *  Check if we have need to sycnrhonize with the engine.
   */
  bool isSynchronous()
  {
    return isSynchro;    
  }
  
  /**
   *  Set if requests synchronize with the engine update or not.
   */
  void setSynchronousMode(bool f)
  {
    isSynchro=f;    
  }
};

#endif
