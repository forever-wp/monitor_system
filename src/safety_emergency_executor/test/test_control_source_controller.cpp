#include <gtest/gtest.h>

#include "safety_emergency_executor/control_source_controller.hpp"

TEST(ControlSourceControllerTest, DefaultsToNavigation)
{
  safety_emergency_executor::ControlSourceController controller("navigation", false);
  EXPECT_EQ(controller.active_source(), "navigation");
}

TEST(ControlSourceControllerTest, SwitchesBetweenCanonicalSources)
{
  safety_emergency_executor::ControlSourceController controller("navigation", false);
  const auto result = controller.set_active_source("remote");

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.changed);
  EXPECT_EQ(result.active_source, "remote");
  EXPECT_EQ(controller.active_source(), "remote");
}

TEST(ControlSourceControllerTest, RejectsInvalidSources)
{
  safety_emergency_executor::ControlSourceController controller("navigation", false);
  const auto result = controller.set_active_source("invalid");

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.changed);
  EXPECT_EQ(controller.active_source(), "navigation");
}
