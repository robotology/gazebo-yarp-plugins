/*
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <gazebo/test/ServerFixture.hh>
#include <gazebo/test/helper_physics_generator.hh>
#include <gazebo/gazebo.hh>

#include <yarp/conf/environment.h>
#include <yarp/serversql/yarpserversql.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/PolyDriver.h>

#include "process.hpp"
#include <memory>

class ClockTest : public gazebo::ServerFixture,
                  public testing::WithParamInterface<const char*>
{
  struct PluginTestHelperOptions
  {
  };

  public: gazebo::event::ConnectionPtr updateConnection;
  public: std::unique_ptr<TinyProcessLib::Process> yarpserverProcess;

  public: void PluginTest(const std::string &_physicsEngine);
  public: void PluginTestHelper(const std::string &_physicsEngine,
                                const std::string &worldName,
                                const PluginTestHelperOptions& options);

            
};

void ClockTest::PluginTestHelper(const std::string &_physicsEngine,
                                             const std::string &worldName,
                                             const PluginTestHelperOptions& options)
{
  // Set YARP_CLOCK to /clock and check if test works
  // Regression test for https://github.com/robotology/gazebo-yarp-plugins/issues/526
  yarp::conf::environment::setEnvironment("YARP_CLOCK", "/clock");

  // Load plugin libgazebo_yarp_clock
  gazebo::addPlugin("libgazebo_yarp_clock.so");


  bool worldPaused = true;
  std::string worldAbsPath = CMAKE_CURRENT_SOURCE_DIR"/" + worldName;
  Load(worldAbsPath, worldPaused, _physicsEngine);

  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  gazebo::physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  gzdbg << "ClockTest: testing world " << worldName << " with physics engine " << _physicsEngine << std::endl;

  // Run a few step of simulation to ensure that YARP plugin start correctly
  world->Step(10);

  // Check if the /clock port has been correctly created
  EXPECT_TRUE(yarp::os::NetworkBase::exists("/clock"));

  // Unload the simulation
  Unload();

  // Close yarpserver
  yarpserverProcess->kill();
}

/////////////////////////////////////////////////////////////////////
void ClockTest::PluginTest(const std::string &_physicsEngine)
{
  // In this case we do not use setLocalMode as we need to 
  // ensure that the low level of YARP behave like in the case 
  // of when the user uses them, i.e. using an external yarpserver
  std::string yarpserverLocation = YARP_SERVER_LOCATION;
  gzdbg << "ClockTest: launching yarpserver from " << YARP_SERVER_LOCATION << std::endl;
  yarpserverProcess = 
    std::make_unique<TinyProcessLib::Process>(yarpserverLocation);
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // Defined by CMake
  std::string pluginDir = GAZEBO_YARP_PLUGINS_DIR;
  gazebo::common::SystemPaths::Instance()->AddPluginPaths(pluginDir);
  std::string modelDir = CMAKE_CURRENT_SOURCE_DIR;
  gazebo::common::SystemPaths::Instance()->AddModelPaths(modelDir);

  PluginTestHelperOptions options;
  this->PluginTestHelper(_physicsEngine, "empty.world", options);
}

TEST_P(ClockTest, PluginTest)
{
  PluginTest(GetParam());
}

// Only test for ode
INSTANTIATE_TEST_CASE_P(PhysicsEngines, ClockTest, ::testing::Values("ode"));

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
