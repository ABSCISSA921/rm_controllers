#include <gtest/gtest.h>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_loader.hpp>
#include <ros/init.h>
#include <string>

TEST(ChassisPluginLoad, CommonControllersDoNotLoadBipedalLibrary)
{
  pluginlib::ClassLoader<controller_interface::ControllerBase> loader(
      "controller_interface", "controller_interface::ControllerBase");
  for (const char* name : {"rm_chassis_controllers/OmniController", "rm_chassis_controllers/SentryController",
                           "rm_chassis_controllers/SwerveController", "rm_chassis_controllers/BalanceController"})
  {
    auto instance = loader.createInstance(name);
    ASSERT_TRUE(instance);
  }
  EXPECT_FALSE(loader.isClassLoaded("rm_chassis_controllers/BipedalController"));
  EXPECT_FALSE(loader.isClassLoaded("rm_chassis_controllers/VMCController"));
}

#if RM_TEST_BUILD_BIPEDAL
TEST(ChassisPluginLoad, BipedalControllersLoadFromSeparateLibrary)
{
  pluginlib::ClassLoader<controller_interface::ControllerBase> loader(
      "controller_interface", "controller_interface::ControllerBase");
  for (const char* name : {"rm_chassis_controllers/BipedalController", "rm_chassis_controllers/VMCController"})
  {
    const std::string path = loader.getClassLibraryPath(name);
    EXPECT_EQ(path.substr(path.find_last_of('/') + 1), "librm_bipedal_controllers.so");
    auto instance = loader.createInstance(name);
    ASSERT_TRUE(instance);
  }
}
#endif

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "chassis_plugin_load_test", ros::init_options::AnonymousName);
  return RUN_ALL_TESTS();
}
