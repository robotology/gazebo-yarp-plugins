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

class ControlBoardControlTest : public gazebo::ServerFixture,
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

void ControlBoardControlTest::PluginTestHelper(const std::string &_physicsEngine,
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

  gzdbg << "ControlBoardControlTest: testing world " << worldName << " with physics engine " << _physicsEngine << std::endl;

  // Get model
  auto model = world->ModelByName("pendulum_with_base");

  std::string jointName = "upper_joint";

  // Set zero gravity to simplify the analysis
  world->SetGravity(ignition::math::Vector3d::Zero);

  // Get joint
  auto joint = model->GetJoint(jointName);
  std::string jointScopedName = joint->GetScopedName();

  // Run a few step of simulation to ensure that YARP plugin start correctly
  world->Step(10);

  // Now, open yarp device
  yarp::os::Property option;
  yarp::dev::PolyDriver        driver;
  yarp::dev::IPositionControl *ipos = 0;
  yarp::dev::IControlMode     *imod = 0;
  yarp::dev::IEncoders        *ienc = 0;
  yarp::dev::IVelocityControl *ivel = 0;
  option.put("device","remote_controlboard");
  option.put("remote","/pendulumGazebo/body");
  option.put("local","/ControlBoardControlTest");

  ASSERT_TRUE(driver.open(option));

  // open the views
  ASSERT_TRUE(driver.view(ipos));
  ASSERT_TRUE(driver.view(imod));
  ASSERT_TRUE(driver.view(ienc));
  ASSERT_TRUE(driver.view(ivel));

  // retrieve number of axes
  int nAxes = 0;
  ienc->getAxes(&nAxes);

  EXPECT_EQ(nAxes, 1);
  double rad2deg = 180.0/3.1415;

  // Set control mode to position
  ASSERT_TRUE(imod->setControlMode(0, VOCAB_CM_POSITION));

  // Step the simulation
  world->Step(10);

  // Check control mode
  int controlMode = 0;
  EXPECT_TRUE(imod->getControlMode(0, &controlMode));
  EXPECT_EQ(controlMode, VOCAB_CM_POSITION);

  // Set a desired velocity and log velocity and positions
  double desiredPositionInDeg = 2.0;
  EXPECT_TRUE(ipos->positionMove(0, desiredPositionInDeg));

  gzdbg << "Setting desired position via YARP to " << desiredPositionInDeg << std::endl;

  for(int i=0; i < 40; i++) {
      world->Step(10);
      /*
      std::cerr << "Desired position:  "  << desiredPositionInDeg << std::endl;
      std::cerr  << "Measured velocity: "  << joint->GetVelocity(0u)*rad2deg << std::endl;
      std::cerr  << "Measured position: "  << joint->Position(0u)*rad2deg << std::endl;*/
  }
  
  // Check if the desired position has been reached
  EXPECT_NEAR(joint->Position(0u)*rad2deg, desiredPositionInDeg, 1e-2);

  // Set control mode to velocity
  ASSERT_TRUE(imod->setControlMode(0, VOCAB_CM_VELOCITY));

  // Step the simulation
  world->Step(100);

  // Check control mode
  controlMode = 0;
  ASSERT_TRUE(imod->getControlMode(0, &controlMode));
  ASSERT_EQ(controlMode, VOCAB_CM_VELOCITY);

  // Set a desired velocity and log velocity and positions
  double desiredVelocityInDegPerSec = 2.0;

  gzdbg << "Setting desired velocity via YARP to " << desiredVelocityInDegPerSec << std::endl;

  for(int i=0; i < 40; i++) {
	  // Set inside the loop to avoid speed timeout 
	  EXPECT_TRUE(ivel->velocityMove(0, desiredVelocityInDegPerSec));
    world->Step(10);
    /*
    std::cerr  << "Desired velocity:  "  << desiredVelocityInDegPerSec << std::endl;
    std::cerr  << "Measured velocity: "  << joint->GetVelocity(0u)*rad2deg << std::endl;
    */
  }

  // Check if the desired position has been reached
  EXPECT_NEAR(joint->GetVelocity(0u)*rad2deg, desiredVelocityInDegPerSec, 1e-2);


  // Close driver
  ASSERT_TRUE(driver.close());

  // Unload the simulation
  Unload();
}

/////////////////////////////////////////////////////////////////////
void ControlBoardControlTest::PluginTest(const std::string &_physicsEngine)
{
  // Make sure that the YARP network does not require yarpserver running
  yarp::os::NetworkBase::setLocalMode(true);

  // Defined by CMake
  std::string pluginDir = GAZEBO_YARP_PLUGINS_DIR;
  gazebo::common::SystemPaths::Instance()->AddPluginPaths(pluginDir);
  std::string modelDir = CMAKE_CURRENT_SOURCE_DIR;
  gazebo::common::SystemPaths::Instance()->AddModelPaths(modelDir);

  PluginTestHelperOptions options;
  this->PluginTestHelper(_physicsEngine, "ControlBoardControlTest.world", options);
}

TEST_P(ControlBoardControlTest, PluginTest)
{
  PluginTest(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, ControlBoardControlTest, PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
