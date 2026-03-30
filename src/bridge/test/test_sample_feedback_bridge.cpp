#include <gtest/gtest.h>
#include <sensor_msgs/msg/battery_state.hpp>

#include "bridge/sample_feedback_bridge.hpp"

TEST(SampleFeedbackBridgeTest, ExtractMetricsUsesExampleBatteryFields)
{
  sensor_msgs::msg::BatteryState msg;
  msg.percentage = 0.42F;
  msg.temperature = 31.5F;
  msg.voltage = 25.2F;
  msg.present = true;

  const auto metrics = bridge::SampleFeedbackBridge::extract_metrics(msg);
  ASSERT_EQ(metrics.size(), 3u);
  EXPECT_EQ(metrics[0].name, "battery_percentage");
  EXPECT_NEAR(metrics[0].value, 0.42, 1e-6);
  EXPECT_TRUE(metrics[0].valid);
  EXPECT_EQ(metrics[1].name, "battery_temperature");
  EXPECT_NEAR(metrics[1].value, 31.5, 1e-6);
  EXPECT_EQ(metrics[2].name, "battery_voltage");
  EXPECT_NEAR(metrics[2].value, 25.2, 1e-6);
}
