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

#include <yarp/dev/Drivers.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/PolyDriver.h>

class RobotInterfaceTest : public gazebo::ServerFixture,
                                        public testing::WithParamInterface<const char*>
{
  struct PluginTestHelperOptions
  {
  };

  public: gazebo::event::ConnectionPtr updateConnection;

  public: void PluginTest(const std::string &_physicsEngine);
  public: void PluginTestHelper(const std::string &_physicsEngine,
                                const std::string &worldName,
                                const PluginTestHelperOptions& options);
};

void RobotInterfaceTest::PluginTestHelper(const std::string &_physicsEngine,
                                             const std::string &worldName,
                                             const PluginTestHelperOptions& options)
{
  bool worldPaused = true;
  std::string worldAbsPath = CMAKE_CURRENT_SOURCE_DIR"/" + worldName;
  Load(worldAbsPath, worldPaused, _physicsEngine);

  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  gazebo::physics::PhysicsEnginePtr physics = world->Physics();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  gzdbg << "RobotInterfaceTest: testing world " << worldName << " with physics engine " << _physicsEngine << std::endl;

  // Run a few step of simulation to ensure that YARP plugin start correctly
  world->Step(10);

  // Try to connect to the model controlboard using the wrapper opened by the 
  // gazebo_yarp_robotinterface plugin
  yarp::os::Property option;
  yarp::dev::PolyDriver        driver;
  yarp::dev::IEncoders        *ienc = 0;
  option.put("device","remote_controlboard");
  option.put("remote","/pendulumGazebo/openedByTheRobotInterface");
  option.put("local","/RobotInterfaceTest");

  ASSERT_TRUE(driver.open(option));

  // open the views
  ASSERT_TRUE(driver.view(ienc));

  // retrieve number of axes
  int nAxes = 0;
  ienc->getAxes(&nAxes);
  EXPECT_EQ(nAxes, 1);

  // Unload the simulation
  Unload();
}

/////////////////////////////////////////////////////////////////////
void RobotInterfaceTest::PluginTest(const std::string &_physicsEngine)
{
  // Make sure that the YARP network does not require yarpserver running
  yarp::os::NetworkBase::setLocalMode(true);

  // Defined by CMake
  std::string pluginDir = GAZEBO_YARP_PLUGINS_DIR;
  gazebo::common::SystemPaths::Instance()->AddPluginPaths(pluginDir);
  std::string modelDir = CMAKE_CURRENT_SOURCE_DIR;
  gazebo::common::SystemPaths::Instance()->AddModelPaths(modelDir);

  PluginTestHelperOptions options;
  this->PluginTestHelper(_physicsEngine, "RobotInterfaceTest.world", options);
}

TEST_P(RobotInterfaceTest, PluginTest)
{
  PluginTest(GetParam());
}

// Only test for ode
INSTANTIATE_TEST_CASE_P(PhysicsEngines, RobotInterfaceTest, ::testing::Values("ode"));

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
