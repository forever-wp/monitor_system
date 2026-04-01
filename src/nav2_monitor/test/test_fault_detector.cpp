#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "nav2_monitor/fault_detector.hpp"
#include "nav2_monitor/monitor_data_store.hpp"
#include "nav2_monitor/fault_state_coordinator.hpp"
#include "nav2_monitor/task_fault_config_selector.hpp"
#include "nav2_monitor/task_status_mapper.hpp"
#include "nav2_monitor/task_status_message_adapter.hpp"
#include "master_interfaces/msg/task_status.hpp"

namespace
{

std::string write_temp_config(const std::string & content, const std::string & suffix)
{
  const std::string path = "/tmp/nav2_monitor_fault_detector_" + suffix + ".yaml";
  std::ofstream out(path);
  out << content;
  return path;
}

class FaultDetectorTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST(TaskFaultConfigSelectorTest, ResolvesMappedAndDefaultConfigs)
{
  nav2_monitor::TaskFaultConfigSelector selector;
  selector.configure(
    "/configs/base.yaml",
    {
      {"default", "/configs/default.yaml"},
      {"todoor", "/configs/todoor.yaml"},
      {"elevator", "/configs/elevator.yaml"}
    });

  EXPECT_FALSE(selector.update_current_task("default"));
  EXPECT_EQ(selector.resolve_fault_config_for_task(), "/configs/default.yaml");

  EXPECT_TRUE(selector.update_current_task("todoor"));
  EXPECT_TRUE(selector.has_task_changed());
  EXPECT_EQ(selector.resolve_fault_config_for_task(), "/configs/todoor.yaml");
  selector.clear_task_changed();
  EXPECT_FALSE(selector.has_task_changed());

  EXPECT_TRUE(selector.update_current_task("unknown_task"));
  EXPECT_EQ(selector.resolve_fault_config_for_task(), "/configs/default.yaml");
}

TEST(TaskFaultConfigSelectorTest, FallsBackToBaseFaultConfigWhenDefaultMappingMissing)
{
  nav2_monitor::TaskFaultConfigSelector selector;
  selector.configure(
    "/configs/base.yaml",
    {
      {"todoor", "/configs/todoor.yaml"}
    });

  selector.update_current_task("elevator");
  EXPECT_EQ(selector.resolve_fault_config_for_task(), "/configs/base.yaml");
}

TEST(TaskStatusMapperTest, ResolvesConfiguredCodesToTasks)
{
  nav2_monitor::TaskStatusMapper mapper;
  mapper.configure({
    {"100", "default"},
    {"200", "elevator"},
    {"301", "todoor"}
  });

  EXPECT_EQ(mapper.resolve_task_for_code("100"), "default");
  EXPECT_EQ(mapper.resolve_task_for_code("200"), "elevator");
  EXPECT_EQ(mapper.resolve_task_for_code("301"), "todoor");
  EXPECT_TRUE(mapper.has_mapping_for_code("200"));
  EXPECT_FALSE(mapper.has_mapping_for_code("999"));
}

TEST(TaskStatusMapperTest, TrimsConfiguredAndIncomingCodes)
{
  nav2_monitor::TaskStatusMapper mapper;
  mapper.configure({
    {" 200 ", " elevator "},
    {"300", "todoor"},
    {"403", "default"}
  });

  EXPECT_EQ(mapper.resolve_task_for_code("200"), "elevator");
  EXPECT_EQ(mapper.resolve_task_for_code(" 200 "), "elevator");
  EXPECT_EQ(mapper.resolve_task_for_code("300\n"), "todoor");
  EXPECT_EQ(mapper.resolve_task_for_code("\t403"), "default");
  EXPECT_TRUE(mapper.resolve_task_for_code("").empty());
  EXPECT_TRUE(mapper.resolve_task_for_code("999").empty());
}

TEST(TaskStatusMapperTest, MappedTasksReuseExistingTaskFaultConfigSelection)
{
  nav2_monitor::TaskStatusMapper mapper;
  mapper.configure({
    {"109", "default"},
    {"200", "elevator"},
    {"301", "todoor"}
  });

  nav2_monitor::TaskFaultConfigSelector selector;
  selector.configure(
    "/configs/base.yaml",
    {
      {"default", "/configs/default.yaml"},
      {"todoor", "/configs/todoor.yaml"},
      {"elevator", "/configs/elevator.yaml"}
    });

  EXPECT_TRUE(selector.update_current_task(mapper.resolve_task_for_code("200")));
  EXPECT_EQ(selector.resolve_fault_config_for_task(), "/configs/elevator.yaml");

  EXPECT_TRUE(selector.update_current_task(mapper.resolve_task_for_code("301")));
  EXPECT_EQ(selector.resolve_fault_config_for_task(), "/configs/todoor.yaml");

  EXPECT_TRUE(selector.update_current_task(mapper.resolve_task_for_code("109")));
  EXPECT_EQ(selector.resolve_fault_config_for_task(), "/configs/default.yaml");
}

TEST(TaskStatusMessageAdapterTest, ExtractsTrimmedStatusCodeFromTaskStatusMessage)
{
  master_interfaces::msg::TaskStatus msg;
  msg.status_code = " 200\n";
  msg.task_uuid = "task-123";
  msg.message = "elevator waiting";

  EXPECT_EQ(nav2_monitor::TaskStatusMessageAdapter::extract_code(msg), "200");
}

TEST(TaskStatusMessageAdapterTest, EmptyStatusCodeStaysEmpty)
{
  master_interfaces::msg::TaskStatus msg;
  msg.status_code = " \t ";
  msg.task_uuid = "task-456";
  msg.message = "ignored";

  EXPECT_TRUE(nav2_monitor::TaskStatusMessageAdapter::extract_code(msg).empty());
}

TEST_F(FaultDetectorTest, NodeInactiveTriggersSafetyThenSupervisor)
{
  const std::string config_text = R"(
modules:
  - name: "navigation"
    supervisor: 1
    safety_system: 1
    nodes:
      - "controller_server"
)";
  const std::string config_path = write_temp_config(config_text, "node_inactive");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_node_inactive");
  nav2_monitor::FaultDetector detector(node.get());
  detector.load_config(config_path);

  detector.update_node_status({{"controller_server", false}});
  detector.update_topic_freq({});

  auto faults = detector.detect_faults();

  ASSERT_EQ(faults.size(), 2u);
  EXPECT_EQ(faults[0].action, nav2_monitor::ActionType::SAFETY_SYSTEM);
  EXPECT_EQ(faults[1].action, nav2_monitor::ActionType::SUPERVISOR);
  EXPECT_EQ(faults[0].level, nav2_monitor::FaultLevel::CRITICAL);
  EXPECT_EQ(faults[1].level, nav2_monitor::FaultLevel::CRITICAL);
  EXPECT_EQ(faults[0].reason, "Node inactive");
  EXPECT_EQ(faults[1].reason, "Node inactive");

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, TopicLowTriggersSupervisorOnly)
{
  const std::string config_text = R"(
modules:
  - name: "navigation"
    supervisor: 1
    safety_system: 1
    nodes:
      - "controller_server"
    watch_topics:
      - name: "/cmd_vel"
        min_hz: 5.0
)";
  const std::string config_path = write_temp_config(config_text, "topic_low");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_topic_low");
  nav2_monitor::FaultDetector detector(node.get());
  detector.load_config(config_path);

  detector.update_node_status({{"controller_server", true}});
  detector.update_topic_freq({{"/cmd_vel", 1.0}});

  auto faults_first = detector.detect_faults();
  EXPECT_TRUE(faults_first.empty());

  auto faults = detector.detect_faults();

  ASSERT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].action, nav2_monitor::ActionType::SUPERVISOR);
  EXPECT_EQ(faults[0].level, nav2_monitor::FaultLevel::ERROR);
  EXPECT_NE(faults[0].reason.find("Topic frequency low"), std::string::npos);

  detector.update_topic_freq({{"/cmd_vel", 10.0}});
  auto faults_recover_pending = detector.detect_faults();
  EXPECT_EQ(faults_recover_pending.size(), 1u);
  auto faults_recovered = detector.detect_faults();
  EXPECT_TRUE(faults_recovered.empty());

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, LegacyTopicsUseIndependentFaultKeysPerTopic)
{
  const std::string config_text = R"(
modules:
  - name: "navigation"
    supervisor: 1
    safety_system: 0
    nodes:
      - "controller_server"
    watch_topics:
      - name: "/cmd_vel"
        min_hz: 5.0
      - name: "/plan"
        min_hz: 1.0
)";
  const std::string config_path = write_temp_config(config_text, "legacy_topic_keys");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_legacy_topic_keys");
  nav2_monitor::FaultDetector detector(node.get());
  detector.load_config(config_path);

  detector.update_node_status({{"controller_server", true}});
  detector.update_topic_freq({{"/cmd_vel", 1.0}, {"/plan", 0.1}});

  auto faults_first = detector.detect_faults();
  EXPECT_TRUE(faults_first.empty());

  auto faults = detector.detect_faults();
  ASSERT_EQ(faults.size(), 2u);
  EXPECT_NE(faults[0].fault_key, faults[1].fault_key);
  EXPECT_NE(faults[0].fault_key.find("topic_legacy:/cmd_vel"), std::string::npos);
  EXPECT_NE(faults[1].fault_key.find("topic_legacy:/plan"), std::string::npos);

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, WatchTopicWithoutThresholdOnlyChecksPresence)
{
  const std::string config_text = R"(
modules:
  - name: "navigation"
    supervisor: 1
    safety_system: 0
    nodes:
      - "controller_server"
    watch_topics:
      - name: "/cmd_vel"
)";
  const std::string config_path = write_temp_config(config_text, "watch_topic_without_threshold");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_watch_topic_without_threshold");
  nav2_monitor::FaultDetector detector(node.get());
  nav2_monitor::MonitorDataStore store;
  detector.load_config(config_path);

  const auto now = node->now();
  store.mark_node_seen("controller_server", now);
  store.set_watch_topic_publisher("/cmd_vel", true);

  auto faults = detector.detect_faults(store, now);
  EXPECT_TRUE(faults.empty());
  EXPECT_FALSE(detector.is_watch_topic_frequency_required("/cmd_vel"));

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, HealthyModuleHasNoFaults)
{
  const std::string config_text = R"(
modules:
  - name: "navigation"
    supervisor: 1
    safety_system: 1
    nodes:
      - "controller_server"
    watch_topics:
      - name: "/cmd_vel"
        min_hz: 5.0
)";
  const std::string config_path = write_temp_config(config_text, "healthy");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_healthy");
  nav2_monitor::FaultDetector detector(node.get());
  detector.load_config(config_path);

  detector.update_node_status({{"controller_server", true}});
  detector.update_topic_freq({{"/cmd_vel", 10.0}});

  auto faults = detector.detect_faults();
  EXPECT_TRUE(faults.empty());

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, TargetTransformsAreLoadedFromFaultConfig)
{
  const std::string config_text = R"(
target_transforms:
  - "map->odom"
  - "odom->base_link"
modules:
  - name: "navigation"
    supervisor: 1
    safety_system: 0
    nodes:
      - "controller_server"
)";
  const std::string config_path = write_temp_config(config_text, "target_transforms_from_fault_config");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_target_transforms");
  nav2_monitor::FaultDetector detector(node.get());
  detector.load_config(config_path);

  EXPECT_EQ(
    detector.get_monitored_transforms(),
    (std::vector<std::string>{"map->odom", "odom->base_link"}));

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, MonitorTargetsAreAggregatedFromModules)
{
  const std::string config_text = R"(
multi_value_judge:
  trigger_count: 3
  recover_count: 4
modules:
  - name: "navigation"
    supervisor: 1
    safety_system: 1
    nodes:
      - "controller_server"
      - "planner_server"
    watch_topics:
      - name: "/cmd_vel"
        min_hz: 5.0
  - name: "localization"
    supervisor: 1
    safety_system: 0
    nodes:
      - "planner_server"
      - "amcl"
    watch_topics:
      - name: "/amcl_pose"
        min_hz: 10.0
      - name: "/cmd_vel"
        min_hz: 1.0
)";
  const std::string config_path = write_temp_config(config_text, "module_aggregate");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_module_aggregate");
  nav2_monitor::FaultDetector detector(node.get());
  detector.load_config(config_path);

  EXPECT_TRUE(detector.has_module_configs());
  EXPECT_EQ(detector.get_multi_value_judge_config().trigger_count, 3u);
  EXPECT_EQ(detector.get_multi_value_judge_config().recover_count, 4u);
  EXPECT_EQ(
    detector.get_monitored_nodes(),
    (std::vector<std::string>{"controller_server", "planner_server", "amcl"}));
  EXPECT_EQ(
    detector.get_watched_topics(),
    (std::vector<std::string>{"/cmd_vel", "/amcl_pose"}));

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, InvalidReloadKeepsLastValidConfig)
{
  const std::string config_text = R"(
modules:
  - name: "navigation"
    supervisor: 1
    safety_system: 1
    nodes:
      - "controller_server"
    watch_topics:
      - name: "/cmd_vel"
        min_hz: 5.0
)";
  const std::string config_path = write_temp_config(config_text, "keep_last_valid");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_keep_last_valid");
  nav2_monitor::FaultDetector detector(node.get());
  detector.load_config(config_path);

  EXPECT_TRUE(detector.has_module_configs());
  EXPECT_EQ(detector.get_monitored_nodes(), (std::vector<std::string>{"controller_server"}));
  EXPECT_EQ(detector.get_watched_topics(), (std::vector<std::string>{"/cmd_vel"}));

  detector.load_config("/tmp/this_config_file_should_not_exist_12345.yaml");

  EXPECT_TRUE(detector.has_module_configs());
  EXPECT_EQ(detector.get_monitored_nodes(), (std::vector<std::string>{"controller_server"}));
  EXPECT_EQ(detector.get_watched_topics(), (std::vector<std::string>{"/cmd_vel"}));

  detector.update_node_status({{"controller_server", false}});
  detector.update_topic_freq({{"/cmd_vel", 10.0}});
  auto faults = detector.detect_faults();
  EXPECT_FALSE(faults.empty());

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, DeprecatedTopicsFieldIsIgnored)
{
  const std::string config_text = R"(
modules:
  - name: "navigation"
    supervisor: 1
    safety_system: 0
    nodes:
      - "controller_server"
    topics:
      - name: "/cmd_vel"
        min_hz: 5.0
)";
  const std::string config_path = write_temp_config(config_text, "deprecated_topics_field");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_deprecated_topics_field");
  nav2_monitor::FaultDetector detector(node.get());
  detector.load_config(config_path);

  EXPECT_TRUE(detector.get_watched_topics().empty());
  detector.update_node_status({{"controller_server", true}});
  detector.update_topic_freq({{"/cmd_vel", 0.0}});
  auto faults = detector.detect_faults();
  EXPECT_TRUE(faults.empty());

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, DeprecatedFeedbackFieldsAreIgnored)
{
  const std::string config_text = R"(
modules:
  - name: "navigation"
    supervisor: 1
    safety_system: 1
    nodes:
      - "controller_server"
    feedback_topics:
      - topic_name: "/controller/feedback"
        metric_name: "tracking_error"
        min_value: 0.0
        max_value: 0.5
        level: "CRITICAL"
        actions: ["safety_system"]
)";
  const std::string config_path = write_temp_config(config_text, "deprecated_feedback_fields");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_deprecated_feedback_fields");
  nav2_monitor::FaultDetector detector(node.get());
  detector.load_config(config_path);

  detector.update_node_status({{"controller_server", true}});
  detector.update_topic_freq({});
  detector.update_feedback_sample(
    "navigation", "/controller/feedback", "tracking_error", 0.9, true, node->now());
  auto faults_first = detector.detect_faults();
  auto faults_second = detector.detect_faults();
  EXPECT_TRUE(faults_first.empty());
  EXPECT_TRUE(faults_second.empty());

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, FeedbackValueOutOfRangeUsesRuleActionsAndLevel)
{
  const std::string config_text = R"(
modules:
  - name: "navigation"
    supervisor: 1
    safety_system: 1
    nodes:
      - "controller_server"
    feedback_rules:
      - source_topic: "/controller/feedback"
        metric_name: "tracking_error"
        min_value: 0.0
        max_value: 0.5
        max_stale_s: 2.0
        level: "CRITICAL"
        actions: ["safety_system", "supervisor"]
)";
  const std::string config_path = write_temp_config(config_text, "feedback_range");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_feedback_range");
  nav2_monitor::FaultDetector detector(node.get());
  detector.set_feedback_default_max_stale(2.0);
  detector.load_config(config_path);

  detector.update_node_status({{"controller_server", true}});
  detector.update_topic_freq({});
  detector.update_feedback_sample(
    "navigation", "/controller/feedback", "tracking_error", 0.9, true, node->now());

  auto faults_first = detector.detect_faults();
  EXPECT_TRUE(faults_first.empty());

  auto faults = detector.detect_faults();

  ASSERT_EQ(faults.size(), 2u);
  EXPECT_EQ(faults[0].level, nav2_monitor::FaultLevel::CRITICAL);
  EXPECT_EQ(faults[1].level, nav2_monitor::FaultLevel::CRITICAL);
  EXPECT_EQ(faults[0].action, nav2_monitor::ActionType::SAFETY_SYSTEM);
  EXPECT_EQ(faults[1].action, nav2_monitor::ActionType::SUPERVISOR);
  EXPECT_NE(faults[0].fault_key.find("feedback:/controller/feedback:tracking_error"), std::string::npos);

  detector.update_feedback_sample(
    "navigation", "/controller/feedback", "tracking_error", 0.1, true, node->now());
  auto faults_recover_pending = detector.detect_faults();
  EXPECT_EQ(faults_recover_pending.size(), 2u);

  detector.update_feedback_sample(
    "navigation", "/controller/feedback", "tracking_error", 0.1, true, node->now());
  auto faults_recovered = detector.detect_faults();
  EXPECT_TRUE(faults_recovered.empty());

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, FeedbackInvalidAndLowFrequencyTriggerFaults)
{
  const std::string config_text = R"(
modules:
  - name: "navigation"
    supervisor: 1
    safety_system: 1
    nodes:
      - "controller_server"
    feedback_rules:
      - source_topic: "/controller/feedback"
        metric_name: "health_score"
        min_value: 0.0
        max_value: 1.0
        min_hz: 10.0
        max_stale_s: 2.0
        level: "ERROR"
        actions: ["supervisor"]
)";
  const std::string config_path = write_temp_config(config_text, "feedback_invalid_freq");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_feedback_invalid_freq");
  nav2_monitor::FaultDetector invalid_detector(node.get());
  invalid_detector.load_config(config_path);
  invalid_detector.update_node_status({{"controller_server", true}});
  invalid_detector.update_topic_freq({});

  invalid_detector.update_feedback_sample(
    "navigation", "/controller/feedback", "health_score", 0.7, false, node->now());
  auto faults_invalid_first = invalid_detector.detect_faults();
  EXPECT_TRUE(faults_invalid_first.empty());
  auto faults_invalid_second = invalid_detector.detect_faults();
  ASSERT_EQ(faults_invalid_second.size(), 1u);
  EXPECT_EQ(faults_invalid_second[0].level, nav2_monitor::FaultLevel::ERROR);
  EXPECT_EQ(faults_invalid_second[0].action, nav2_monitor::ActionType::SUPERVISOR);

  nav2_monitor::FaultDetector low_freq_detector(node.get());
  low_freq_detector.load_config(config_path);
  low_freq_detector.update_node_status({{"controller_server", true}});
  low_freq_detector.update_topic_freq({});

  low_freq_detector.update_feedback_sample(
    "navigation", "/controller/feedback", "health_score", 0.7, true,
    node->now() - rclcpp::Duration::from_seconds(1.0));
  auto faults_low_freq_first = low_freq_detector.detect_faults();
  EXPECT_TRUE(faults_low_freq_first.empty());
  auto faults_low_freq_second = low_freq_detector.detect_faults();
  ASSERT_EQ(faults_low_freq_second.size(), 1u);
  EXPECT_EQ(faults_low_freq_second[0].level, nav2_monitor::FaultLevel::ERROR);
  EXPECT_EQ(faults_low_freq_second[0].action, nav2_monitor::ActionType::SUPERVISOR);

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, InvalidFeedbackRulesAreSkipped)
{
  const std::string config_text = R"(
modules:
  - name: "navigation"
    supervisor: 1
    safety_system: 1
    nodes:
      - "controller_server"
    feedback_rules:
      - source_topic: "/controller/feedback"
        metric_name: "health_score"
        level: "ERROR"
        actions: []
      - source_topic: "/controller/feedback"
        metric_name: "health_score2"
        min_value: 1.0
        max_value: 0.0
        level: "ERROR"
        actions: ["supervisor"]
)";
  const std::string config_path = write_temp_config(config_text, "feedback_invalid_rules");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_feedback_invalid_rules");
  nav2_monitor::FaultDetector detector(node.get());
  detector.load_config(config_path);

  detector.update_node_status({{"controller_server", true}});
  detector.update_topic_freq({});
  auto faults = detector.detect_faults();
  EXPECT_TRUE(faults.empty());

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, MissingFeedbackAfterStaleTriggersFault)
{
  const std::string config_text = R"(
modules:
  - name: "navigation"
    supervisor: 1
    safety_system: 1
    nodes:
      - "controller_server"
    feedback_rules:
      - source_topic: "/controller/feedback"
        metric_name: "health_score"
        min_value: 0.0
        max_value: 1.0
        max_stale_s: 0.0
        level: "ERROR"
        actions: ["supervisor"]
)";
  const std::string config_path = write_temp_config(config_text, "feedback_missing");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_feedback_missing");
  nav2_monitor::FaultDetector detector(node.get());
  detector.load_config(config_path);

  detector.update_node_status({{"controller_server", true}});
  detector.update_topic_freq({});

  auto faults_first = detector.detect_faults();
  auto faults_second = detector.detect_faults();
  auto faults_third = detector.detect_faults();

  EXPECT_TRUE(faults_first.empty() || faults_second.empty());
  const auto & faults = faults_third.empty() ? faults_second : faults_third;
  ASSERT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].action, nav2_monitor::ActionType::SUPERVISOR);
  EXPECT_EQ(faults[0].level, nav2_monitor::FaultLevel::ERROR);

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, MultiMetricOnlyFaultsOnAbnormalMetric)
{
  const std::string config_text = R"(
modules:
  - name: "navigation"
    supervisor: 1
    safety_system: 1
    nodes:
      - "controller_server"
    feedback_rules:
      - source_topic: "/controller/feedback"
        metric_name: "metric_a"
        min_value: 0.0
        max_value: 1.0
        max_stale_s: 5.0
        level: "ERROR"
        actions: ["supervisor"]
      - source_topic: "/controller/feedback"
        metric_name: "metric_b"
        min_value: 0.0
        max_value: 1.0
        max_stale_s: 5.0
        level: "CRITICAL"
        actions: ["safety_system", "supervisor"]
)";
  const std::string config_path = write_temp_config(config_text, "feedback_multimetric");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_feedback_multimetric");
  nav2_monitor::FaultDetector detector(node.get());
  detector.load_config(config_path);

  detector.update_node_status({{"controller_server", true}});
  detector.update_topic_freq({});

  detector.update_feedback_sample("navigation", "/controller/feedback", "metric_a", 0.8, true, node->now());
  detector.update_feedback_sample("navigation", "/controller/feedback", "metric_b", 2.0, true, node->now());

  auto faults_first = detector.detect_faults();
  EXPECT_TRUE(faults_first.empty());

  auto faults = detector.detect_faults();
  ASSERT_EQ(faults.size(), 2u);
  EXPECT_EQ(faults[0].action, nav2_monitor::ActionType::SAFETY_SYSTEM);
  EXPECT_EQ(faults[1].action, nav2_monitor::ActionType::SUPERVISOR);
  EXPECT_EQ(faults[0].level, nav2_monitor::FaultLevel::CRITICAL);
  EXPECT_EQ(faults[1].level, nav2_monitor::FaultLevel::CRITICAL);

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, CollisionDetectionApproachTriggersSlowDownByTTC)
{
  const std::string config_text = R"(
collision_detection:
  enabled: 1
  module_name: "collision_detection"
  scan_topic: "/scan"
  source_timeout_s: 1.0
  zones:
    - name: "front_approach"
      enabled: 1
      model: "approach"
      points: [1.2, 0.4, 1.2, -0.4, 0.2, -0.4, 0.2, 0.4]
      level: "WARNING"
      safety_system: 1
      safety_slow_down_percentage: 40.0
      time_before_collision: 1.0
      actions: ["safety_system"]
modules:
  - name: "dummy"
    supervisor: 0
    safety_system: 0
)";
  const std::string config_path = write_temp_config(config_text, "collision_detection_approach");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_collision_approach");
  nav2_monitor::FaultDetector detector(node.get());
  nav2_monitor::MonitorDataStore store;
  detector.load_config(config_path);

  const auto now = node->now();
  store.set_prediction_speed(1.0, now);
  store.set_collision_points("scan", {
    nav2_monitor::CollisionPoint{0.5, 0.0},
    nav2_monitor::CollisionPoint{0.6, 0.1}
  }, now);

  auto faults_first = detector.detect_faults(store, now);
  EXPECT_TRUE(faults_first.empty());

  auto faults = detector.detect_faults(store, now);
  ASSERT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].safety_command, nav2_monitor::SafetyCommandType::SLOW_DOWN);
  EXPECT_NE(faults[0].reason.find("ttc="), std::string::npos);

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, CollisionDetectionApproachUsesPredictedSpeedNotFinalCommandSpeed)
{
  const std::string config_text = R"(
collision_detection:
  enabled: 1
  module_name: "collision_detection"
  prediction_speed_topic: "/cmd_vel"
  scan_topic: "/scan"
  source_timeout_s: 1.0
  zones:
    - name: "front_approach"
      enabled: 1
      model: "approach"
      points: [1.2, 0.4, 1.2, -0.4, 0.2, -0.4, 0.2, 0.4]
      level: "WARNING"
      safety_system: 1
      safety_slow_down_percentage: 40.0
      time_before_collision: 1.0
      actions: ["safety_system"]
modules:
  - name: "dummy"
    supervisor: 0
    safety_system: 0
)";
  const std::string config_path = write_temp_config(config_text, "collision_detection_predicted_speed");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_collision_predicted_speed");
  nav2_monitor::FaultDetector detector(node.get());
  nav2_monitor::MonitorDataStore store;
  detector.load_config(config_path);

  const auto now = node->now();
  store.set_command_speed(0.1, now);
  store.set_prediction_speed(1.0, now);
  store.set_collision_points("scan", {
    nav2_monitor::CollisionPoint{0.5, 0.0}
  }, now);

  auto faults_first = detector.detect_faults(store, now);
  EXPECT_TRUE(faults_first.empty());

  auto faults = detector.detect_faults(store, now);
  ASSERT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].safety_command, nav2_monitor::SafetyCommandType::SLOW_DOWN);
  EXPECT_NE(faults[0].reason.find("ttc="), std::string::npos);

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, CollisionDetectionApproachUsesFootprintClearance)
{
  const std::string config_text = R"(
collision_detection:
  enabled: 1
  module_name: "collision_detection"
  scan_topic: "/scan"
  source_timeout_s: 1.0
  footprint_points: [-0.37, -0.28, -0.37, 0.28, 0.37, 0.28, 0.37, -0.28]
  zones:
    - name: "front_approach"
      enabled: 1
      model: "approach"
      points: [1.3, 0.4, 1.3, -0.4, 0.2, -0.4, 0.2, 0.4]
      level: "WARNING"
      safety_system: 1
      safety_slow_down_percentage: 40.0
      time_before_collision: 1.0
      actions: ["safety_system"]
modules:
  - name: "dummy"
    supervisor: 0
    safety_system: 0
)";
  const std::string config_path = write_temp_config(config_text, "collision_detection_footprint_ttc");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_collision_footprint_ttc");
  nav2_monitor::FaultDetector detector(node.get());
  nav2_monitor::MonitorDataStore store;
  detector.load_config(config_path);

  const auto now = node->now();
  store.set_prediction_speed(1.0, now);
  store.set_collision_points("scan", {
    nav2_monitor::CollisionPoint{1.2, 0.0}
  }, now);

  auto faults_first = detector.detect_faults(store, now);
  EXPECT_TRUE(faults_first.empty());

  auto faults = detector.detect_faults(store, now);
  ASSERT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].safety_command, nav2_monitor::SafetyCommandType::SLOW_DOWN);
  EXPECT_NE(faults[0].reason.find("ttc="), std::string::npos);
  EXPECT_NE(faults[0].reason.find("clearance="), std::string::npos);

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, CollisionDetectionApproachUsesTrajectoryDirection)
{
  const std::string config_text = R"(
collision_detection:
  enabled: 1
  module_name: "collision_detection"
  scan_topic: "/scan"
  source_timeout_s: 1.0
  footprint_points: [-0.37, -0.28, -0.37, 0.28, 0.37, 0.28, 0.37, -0.28]
  zones:
    - name: "front_approach"
      enabled: 1
      model: "approach"
      points: [1.3, 0.4, 1.3, -0.4, 0.2, -0.4, 0.2, 0.4]
      level: "WARNING"
      safety_system: 1
      safety_slow_down_percentage: 40.0
      time_before_collision: 1.0
      simulation_time_step: 0.1
      actions: ["safety_system"]
modules:
  - name: "dummy"
    supervisor: 0
    safety_system: 0
)";
  const std::string config_path = write_temp_config(config_text, "collision_detection_trajectory_ttc");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_collision_trajectory_ttc");
  nav2_monitor::FaultDetector detector(node.get());
  nav2_monitor::MonitorDataStore store;
  detector.load_config(config_path);

  const auto now = node->now();
  store.set_prediction_motion(-1.0, 0.0, 0.0, now);
  store.set_collision_points("scan", {
    nav2_monitor::CollisionPoint{0.5, 0.0}
  }, now);

  auto faults_first = detector.detect_faults(store, now);
  EXPECT_TRUE(faults_first.empty());

  auto faults = detector.detect_faults(store, now);
  EXPECT_TRUE(faults.empty());

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, CollisionDetectionApproachUsesRecoverThresholdHysteresis)
{
  const std::string config_text = R"(
collision_detection:
  enabled: 1
  module_name: "collision_detection"
  scan_topic: "/scan"
  source_timeout_s: 1.0
  footprint_points: [-0.37, -0.28, -0.37, 0.28, 0.37, 0.28, 0.37, -0.28]
  zones:
    - name: "front_approach"
      enabled: 1
      model: "approach"
      points: [2.5, 0.4, 2.5, -0.4, 0.2, -0.4, 0.2, 0.4]
      level: "WARNING"
      safety_system: 1
      safety_slow_down_percentage: 40.0
      time_before_collision: 1.0
      recover_time_before_collision: 2.0
      simulation_time_step: 0.1
      actions: ["safety_system"]
modules:
  - name: "dummy"
    supervisor: 0
    safety_system: 0
)";
  const std::string config_path = write_temp_config(config_text, "collision_detection_hysteresis_ttc");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_collision_hysteresis_ttc");
  nav2_monitor::FaultDetector detector(node.get());
  nav2_monitor::MonitorDataStore store;
  detector.load_config(config_path);

  const auto now = node->now();
  store.set_prediction_motion(1.0, 0.0, 0.0, now);
  store.set_collision_points("scan", {
    nav2_monitor::CollisionPoint{1.2, 0.0}
  }, now);
  EXPECT_TRUE(detector.detect_faults(store, now).empty());
  auto faults = detector.detect_faults(store, now);
  ASSERT_EQ(faults.size(), 1u);

  const auto near_recovery_time = now + rclcpp::Duration::from_seconds(0.2);
  store.set_collision_points("scan", {
    nav2_monitor::CollisionPoint{2.0, 0.0}
  }, near_recovery_time);
  auto hysteresis_faults = detector.detect_faults(store, near_recovery_time);
  EXPECT_EQ(hysteresis_faults.size(), 1u);

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, CollisionDetectionApproachUsesMinHoldTime)
{
  const std::string config_text = R"(
collision_detection:
  enabled: 1
  module_name: "collision_detection"
  scan_topic: "/scan"
  source_timeout_s: 1.0
  footprint_points: [-0.37, -0.28, -0.37, 0.28, 0.37, 0.28, 0.37, -0.28]
  zones:
    - name: "front_approach"
      enabled: 1
      model: "approach"
      points: [2.5, 0.4, 2.5, -0.4, 0.2, -0.4, 0.2, 0.4]
      level: "WARNING"
      safety_system: 1
      safety_slow_down_percentage: 40.0
      time_before_collision: 1.0
      recover_time_before_collision: 2.0
      min_hold_time_s: 0.5
      simulation_time_step: 0.1
      actions: ["safety_system"]
modules:
  - name: "dummy"
    supervisor: 0
    safety_system: 0
)";
  const std::string config_path = write_temp_config(config_text, "collision_detection_hold_ttc");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_collision_hold_ttc");
  nav2_monitor::FaultDetector detector(node.get());
  nav2_monitor::MonitorDataStore store;
  detector.load_config(config_path);

  const auto now = node->now();
  store.set_prediction_motion(1.0, 0.0, 0.0, now);
  store.set_collision_points("scan", {
    nav2_monitor::CollisionPoint{1.2, 0.0}
  }, now);
  EXPECT_TRUE(detector.detect_faults(store, now).empty());
  auto faults = detector.detect_faults(store, now);
  ASSERT_EQ(faults.size(), 1u);

  const auto early_recovery_time = now + rclcpp::Duration::from_seconds(0.2);
  store.set_collision_points("scan", {
    nav2_monitor::CollisionPoint{2.5, 0.0}
  }, early_recovery_time);
  auto hold_faults = detector.detect_faults(store, early_recovery_time);
  EXPECT_EQ(hold_faults.size(), 1u);

  const auto recover_1 = now + rclcpp::Duration::from_seconds(0.6);
  store.set_collision_points("scan", {
    nav2_monitor::CollisionPoint{2.5, 0.0}
  }, recover_1);
  auto pending_recover_faults = detector.detect_faults(store, recover_1);
  EXPECT_EQ(pending_recover_faults.size(), 1u);

  const auto recover_2 = now + rclcpp::Duration::from_seconds(0.7);
  store.set_collision_points("scan", {
    nav2_monitor::CollisionPoint{2.5, 0.0}
  }, recover_2);
  auto recovered_faults = detector.detect_faults(store, recover_2);
  EXPECT_TRUE(recovered_faults.empty());

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, CollisionDetectionSlowdownZoneTriggersSlowDown)
{
  const std::string config_text = R"(
collision_detection:
  enabled: 1
  module_name: "collision_detection"
  scan_topic: "/scan"
  source_timeout_s: 1.0
  zones:
    - name: "front_slow"
      enabled: 1
      points: [1.0, 0.3, 1.0, -0.3, 0.0, -0.3, 0.0, 0.3]
      min_points: 2
      level: "ERROR"
      safety_system: 1
      safety_slow_down_percentage: 35.0
      actions: ["safety_system"]
modules:
  - name: "dummy"
    supervisor: 0
    safety_system: 0
)";
  const std::string config_path = write_temp_config(config_text, "collision_detection_slowdown_zone");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_collision_slowdown_zone");
  nav2_monitor::FaultDetector detector(node.get());
  nav2_monitor::MonitorDataStore store;
  detector.load_config(config_path);

  const auto now = node->now();
  store.set_collision_points("scan", {
    nav2_monitor::CollisionPoint{0.3, 0.0},
    nav2_monitor::CollisionPoint{0.6, 0.1}
  }, now);

  auto faults_first = detector.detect_faults(store, now);
  EXPECT_TRUE(faults_first.empty());

  auto faults = detector.detect_faults(store, now);
  ASSERT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].safety_command, nav2_monitor::SafetyCommandType::SLOW_DOWN);
  EXPECT_DOUBLE_EQ(faults[0].safety_slow_down_percentage, 35.0);

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, CollisionDetectionUltrasonicWidgetUsesBuiltInLayout)
{
  const std::string config_text = R"(
collision_detection:
  enabled: 1
  module_name: "collision_detection"
  ultrasonic_topic: "/ultrasonic_eight"
  ultrasonic_widget: [0.2, 0.9, 0.4, 0.1, 0.1, 0.4, 0.9, 0.2]
modules:
  - name: "dummy"
    supervisor: 0
    safety_system: 0
)";
  const std::string config_path = write_temp_config(config_text, "collision_detection_ultrasonic_widget");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_collision_ultrasonic_widget");
  nav2_monitor::FaultDetector detector(node.get());
  detector.load_config(config_path);

  const auto & cfg = detector.get_collision_detection_config();
  ASSERT_EQ(cfg.ultrasonic_sensors.size(), 8u);
  EXPECT_DOUBLE_EQ(cfg.ultrasonic_sensors[0].weight, 0.2);
  EXPECT_DOUBLE_EQ(cfg.ultrasonic_sensors[1].weight, 0.9);
  EXPECT_DOUBLE_EQ(cfg.ultrasonic_sensors[6].weight, 0.9);
  EXPECT_DOUBLE_EQ(cfg.ultrasonic_sensors[7].weight, 0.2);
  EXPECT_NEAR(cfg.ultrasonic_sensors[1].yaw_deg, 0.0, 1e-9);
  EXPECT_NEAR(cfg.ultrasonic_sensors[0].yaw_deg, 45.0, 1e-9);
  EXPECT_NEAR(cfg.ultrasonic_sensors[5].yaw_deg, 180.0, 1e-9);

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, CollisionDetectionParsesTtcVisualizationEnabled)
{
  const std::string config_text = R"(
collision_detection:
  enabled: 1
  module_name: "collision_detection"
  ttc_visualization_enabled: 1
modules:
  - name: "dummy"
    supervisor: 0
    safety_system: 0
)";
  const std::string config_path = write_temp_config(config_text, "collision_ttc_visualization_enabled");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_collision_ttc_visualization_enabled");
  nav2_monitor::FaultDetector detector(node.get());
  detector.load_config(config_path);

  const auto & cfg = detector.get_collision_detection_config();
  EXPECT_TRUE(cfg.ttc_visualization_enabled);

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, CollisionDetectionTtcVisualizationDefaultsToDisabled)
{
  const std::string config_text = R"(
collision_detection:
  enabled: 1
  module_name: "collision_detection"
modules:
  - name: "dummy"
    supervisor: 0
    safety_system: 0
)";
  const std::string config_path = write_temp_config(config_text, "collision_ttc_visualization_default");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_collision_ttc_visualization_default");
  nav2_monitor::FaultDetector detector(node.get());
  detector.load_config(config_path);

  const auto & cfg = detector.get_collision_detection_config();
  EXPECT_FALSE(cfg.ttc_visualization_enabled);

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, CollisionDetectionTtcVisualizationCapturesTrajectoryWhenEnabled)
{
  const std::string config_text = R"(
multi_value_judge:
  trigger_count: 1
  recover_count: 1
collision_detection:
  enabled: 1
  module_name: "collision_detection"
  source_timeout_s: 1.0
  ttc_visualization_enabled: 1
  footprint_points: [-0.37, -0.28, -0.37, 0.28, 0.37, 0.28, 0.37, -0.28]
  zones:
    - name: "front_approach"
      enabled: 1
      model: "approach"
      points: [1.3, 0.4, 1.3, -0.4, 0.2, -0.4, 0.2, 0.4]
      level: "WARNING"
      safety_system: 1
      safety_slow_down_percentage: 40.0
      time_before_collision: 1.0
      simulation_time_step: 0.1
      actions: ["safety_system"]
modules:
  - name: "dummy"
    supervisor: 0
    safety_system: 0
)";
  const std::string config_path = write_temp_config(config_text, "collision_ttc_visualization_snapshot");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_collision_ttc_visualization_snapshot");
  nav2_monitor::FaultDetector detector(node.get());
  nav2_monitor::MonitorDataStore store;
  detector.load_config(config_path);

  const auto now = node->now();
  store.set_prediction_motion(1.0, 0.0, 0.0, now);
  store.set_collision_points("scan", {
    nav2_monitor::CollisionPoint{1.2, 0.0}
  }, now);

  auto faults = detector.detect_faults(store, now);
  ASSERT_EQ(faults.size(), 1u);

  const auto & vis = detector.get_collision_ttc_visualization();
  EXPECT_TRUE(vis.enabled);
  EXPECT_TRUE(vis.active);
  EXPECT_EQ(vis.zone_name, "front_approach");
  EXPECT_FALSE(vis.trajectory_points.empty());
  EXPECT_FALSE(vis.footprint_samples.empty());

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, CollisionDetectionWeightedUltrasonicPointCanTriggerZone)
{
  const std::string config_text = R"(
collision_detection:
  enabled: 1
  module_name: "collision_detection"
  ultrasonic_topic: "/ultrasonic_eight"
  source_timeout_s: 1.0
  zones:
    - name: "front_stop"
      enabled: 1
      points: [0.5, 0.25, 0.5, -0.25, 0.0, -0.25, 0.0, 0.25]
      min_points: 1.5
      level: "CRITICAL"
      safety_system: 2
      actions: ["safety_system"]
modules:
  - name: "dummy"
    supervisor: 0
    safety_system: 0
)";
  const std::string config_path = write_temp_config(config_text, "collision_detection_weighted_ultrasonic");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_collision_weighted_ultrasonic");
  nav2_monitor::FaultDetector detector(node.get());
  nav2_monitor::MonitorDataStore store;
  detector.load_config(config_path);

  const auto now = node->now();
  store.set_collision_points("ultrasonic", {
    nav2_monitor::CollisionPoint{0.25, 0.0, 1.6}
  }, now);

  auto faults_first = detector.detect_faults(store, now);
  EXPECT_TRUE(faults_first.empty());

  auto faults = detector.detect_faults(store, now);
  ASSERT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].safety_command, nav2_monitor::SafetyCommandType::SOFT_STOP);
  EXPECT_NE(faults[0].reason.find("weighted_points="), std::string::npos);

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, CollisionDetectionAggregatesPointCloudSource)
{
  const std::string config_text = R"(
collision_detection:
  enabled: 1
  module_name: "collision_detection"
  pointcloud_topic: "/points"
  pointcloud_min_height: 0.0
  pointcloud_max_height: 1.0
  source_timeout_s: 1.0
  zones:
    - name: "front_stop"
      enabled: 1
      points: [0.5, 0.2, 0.5, -0.2, 0.0, -0.2, 0.0, 0.2]
      min_points: 2
      level: "CRITICAL"
      safety_system: 2
      actions: ["safety_system"]
modules:
  - name: "dummy"
    supervisor: 0
    safety_system: 0
)";
  const std::string config_path = write_temp_config(config_text, "collision_detection_pointcloud");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_collision_pointcloud");
  nav2_monitor::FaultDetector detector(node.get());
  nav2_monitor::MonitorDataStore store;
  detector.load_config(config_path);

  const auto now = node->now();
  store.set_collision_points("pointcloud", {
    nav2_monitor::CollisionPoint{0.1, 0.0},
    nav2_monitor::CollisionPoint{0.2, 0.1}
  }, now);

  auto faults_first = detector.detect_faults(store, now);
  EXPECT_TRUE(faults_first.empty());

  auto faults = detector.detect_faults(store, now);
  ASSERT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].safety_command, nav2_monitor::SafetyCommandType::SOFT_STOP);

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, CollisionDetectionZoneHitTriggersSafety)
{
  const std::string config_text = R"(
collision_detection:
  enabled: 1
  module_name: "collision_detection"
  scan_topic: "/scan"
  source_timeout_s: 1.0
  zones:
    - name: "front_stop"
      enabled: 1
      points: [0.5, 0.2, 0.5, -0.2, 0.0, -0.2, 0.0, 0.2]
      min_points: 2
      level: "CRITICAL"
      safety_system: 2
      actions: ["safety_system"]
modules:
  - name: "dummy"
    supervisor: 0
    safety_system: 0
)";
  const std::string config_path = write_temp_config(config_text, "collision_detection_zone");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_collision_zone");
  nav2_monitor::FaultDetector detector(node.get());
  nav2_monitor::MonitorDataStore store;
  detector.load_config(config_path);

  const auto now = node->now();
  store.set_collision_points({
    nav2_monitor::CollisionPoint{0.1, 0.0},
    nav2_monitor::CollisionPoint{0.2, 0.1},
    nav2_monitor::CollisionPoint{1.0, 1.0}
  }, now);

  auto faults_first = detector.detect_faults(store, now);
  EXPECT_TRUE(faults_first.empty());

  auto faults = detector.detect_faults(store, now);
  ASSERT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].module_name, "collision_detection");
  EXPECT_EQ(faults[0].action, nav2_monitor::ActionType::SAFETY_SYSTEM);
  EXPECT_EQ(faults[0].safety_command, nav2_monitor::SafetyCommandType::SOFT_STOP);
  EXPECT_NE(faults[0].fault_key.find("collision:front_stop"), std::string::npos);

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, CollisionZoneIgnoresRearStopWhenMovingForward)
{
  const std::string config_text = R"(
multi_value_judge:
  trigger_count: 1
  recover_count: 1
collision_detection:
  enabled: 1
  module_name: "collision_detection"
  source_timeout_s: 1.0
  direction_speed_threshold: 0.05
  zones:
    - name: "rear_stop"
      enabled: 1
      motion_direction: "reverse"
      points: [0.0, 0.25, 0.0, -0.25, -0.45, -0.25, -0.45, 0.25]
      min_points: 1
      level: "CRITICAL"
      safety_system: 2
      actions: ["safety_system"]
modules:
  - name: "dummy"
    supervisor: 0
    safety_system: 0
)";
  const std::string config_path = write_temp_config(config_text, "collision_direction_forward_ignores_rear");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_collision_direction_forward_ignores_rear");
  nav2_monitor::FaultDetector detector(node.get());
  nav2_monitor::MonitorDataStore store;
  detector.load_config(config_path);

  const auto now = node->now();
  store.set_prediction_motion(1.0, 0.0, 0.0, now);
  store.set_collision_points("scan", {
    nav2_monitor::CollisionPoint{-0.2, 0.0}
  }, now);

  auto faults = detector.detect_faults(store, now);
  EXPECT_TRUE(faults.empty());

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, CollisionZoneIgnoresFrontStopWhenReversing)
{
  const std::string config_text = R"(
multi_value_judge:
  trigger_count: 1
  recover_count: 1
collision_detection:
  enabled: 1
  module_name: "collision_detection"
  source_timeout_s: 1.0
  direction_speed_threshold: 0.05
  zones:
    - name: "front_stop"
      enabled: 1
      motion_direction: "forward"
      points: [0.5, 0.25, 0.5, -0.25, 0.0, -0.25, 0.0, 0.25]
      min_points: 1
      level: "CRITICAL"
      safety_system: 2
      actions: ["safety_system"]
modules:
  - name: "dummy"
    supervisor: 0
    safety_system: 0
)";
  const std::string config_path = write_temp_config(config_text, "collision_direction_reverse_ignores_front");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_collision_direction_reverse_ignores_front");
  nav2_monitor::FaultDetector detector(node.get());
  nav2_monitor::MonitorDataStore store;
  detector.load_config(config_path);

  const auto now = node->now();
  store.set_prediction_motion(-1.0, 0.0, 0.0, now);
  store.set_collision_points("scan", {
    nav2_monitor::CollisionPoint{0.2, 0.0}
  }, now);

  auto faults = detector.detect_faults(store, now);
  EXPECT_TRUE(faults.empty());

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, CollisionZoneKeepsLastDirectionWhenSpeedNearZero)
{
  const std::string config_text = R"(
multi_value_judge:
  trigger_count: 1
  recover_count: 1
collision_detection:
  enabled: 1
  module_name: "collision_detection"
  source_timeout_s: 1.0
  direction_speed_threshold: 0.05
  zones:
    - name: "front_stop"
      enabled: 1
      motion_direction: "forward"
      points: [0.5, 0.25, 0.5, -0.25, 0.0, -0.25, 0.0, 0.25]
      min_points: 1
      level: "CRITICAL"
      safety_system: 2
      actions: ["safety_system"]
    - name: "rear_stop"
      enabled: 1
      motion_direction: "reverse"
      points: [0.0, 0.25, 0.0, -0.25, -0.45, -0.25, -0.45, 0.25]
      min_points: 1
      level: "CRITICAL"
      safety_system: 2
      actions: ["safety_system"]
modules:
  - name: "dummy"
    supervisor: 0
    safety_system: 0
)";
  const std::string config_path = write_temp_config(config_text, "collision_direction_hold_last");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_collision_direction_hold_last");
  nav2_monitor::FaultDetector detector(node.get());
  nav2_monitor::MonitorDataStore store;
  detector.load_config(config_path);

  const auto now = node->now();
  store.set_prediction_motion(1.0, 0.0, 0.0, now);
  store.set_collision_points("scan", {
    nav2_monitor::CollisionPoint{2.0, 2.0}
  }, now);
  EXPECT_TRUE(detector.detect_faults(store, now).empty());

  const auto near_zero_time = now + rclcpp::Duration::from_seconds(0.1);
  store.set_prediction_motion(0.01, 0.0, 0.0, near_zero_time);
  store.set_collision_points("scan", {
    nav2_monitor::CollisionPoint{-0.2, 0.0}
  }, near_zero_time);

  auto faults = detector.detect_faults(store, near_zero_time);
  EXPECT_TRUE(faults.empty());

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, ChassisWithoutOdomStillTriggersAnomaly)
{
  const std::string config_text = R"(
chassis_stationary:
  enabled: 1
  odom_topic: ""
  imu_topic: ""
  source_timeout_s: 10.0
  idle_timeout_s: 30.0
  command_speed_threshold: 0.05
  moto_speed_threshold: 0.05
  odom_speed_threshold: 0.03
  anomaly_level: "ERROR"
  idle_level: "WARNING"
  anomaly_actions: ["supervisor"]
  idle_actions: ["none"]
modules:
  - name: "dummy"
    supervisor: 1
    safety_system: 0
)";
  const std::string config_path = write_temp_config(config_text, "chassis_without_odom");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_chassis_without_odom");
  nav2_monitor::FaultDetector detector(node.get());
  detector.load_config(config_path);
  detector.update_command_speed(0.5, node->now());

  auto faults_first = detector.detect_faults();
  EXPECT_TRUE(faults_first.empty());

  auto faults = detector.detect_faults();
  ASSERT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].action, nav2_monitor::ActionType::SUPERVISOR);
  EXPECT_NE(faults[0].reason.find("raw odom disabled"), std::string::npos);

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, ChassisCommandHasMotoMissingTriggersAnomaly)
{
  const std::string config_text = R"(
chassis_stationary:
  enabled: 1
  imu_topic: ""
  source_timeout_s: 10.0
  idle_timeout_s: 30.0
  command_speed_threshold: 0.05
  moto_speed_threshold: 0.05
  odom_speed_threshold: 0.03
  anomaly_level: "ERROR"
  idle_level: "WARNING"
  anomaly_actions: ["supervisor"]
  idle_actions: ["none"]
modules:
  - name: "dummy"
    supervisor: 1
    safety_system: 0
)";
  const std::string config_path = write_temp_config(config_text, "chassis_cmd_has_moto_missing");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_chassis_cmd_has_moto_missing");
  nav2_monitor::FaultDetector detector(node.get());
  detector.load_config(config_path);
  detector.update_command_speed(0.5, node->now());
  detector.update_odom_speed(0.0, node->now());

  auto faults_first = detector.detect_faults();
  EXPECT_TRUE(faults_first.empty());

  auto faults = detector.detect_faults();
  ASSERT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].module_name, "chassis_stationary");
  EXPECT_EQ(faults[0].action, nav2_monitor::ActionType::SUPERVISOR);
  EXPECT_EQ(faults[0].level, nav2_monitor::FaultLevel::ERROR);

  detector.update_moto_speed(0.3, 0.3, node->now(), true);
  auto faults_recover_pending = detector.detect_faults();
  EXPECT_LE(faults_recover_pending.size(), 1u);

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, ChassisCommandMissingMotoHasTriggersAnomaly)
{
  const std::string config_text = R"(
chassis_stationary:
  enabled: 1
  imu_topic: ""
  source_timeout_s: 10.0
  idle_timeout_s: 30.0
  command_speed_threshold: 0.05
  moto_speed_threshold: 0.05
  odom_speed_threshold: 0.03
  anomaly_level: "ERROR"
  idle_level: "WARNING"
  anomaly_actions: ["supervisor"]
  idle_actions: ["none"]
modules:
  - name: "dummy"
    supervisor: 1
    safety_system: 0
)";
  const std::string config_path = write_temp_config(config_text, "chassis_cmd_missing_moto_has");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_chassis_cmd_missing_moto_has");
  nav2_monitor::FaultDetector detector(node.get());
  detector.load_config(config_path);
  detector.update_command_speed(0.0, node->now());
  detector.update_moto_speed(0.2, 0.1, node->now(), true);

  auto faults_first = detector.detect_faults();
  EXPECT_TRUE(faults_first.empty());

  auto faults = detector.detect_faults();
  ASSERT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].module_name, "chassis_stationary");
  EXPECT_EQ(faults[0].action, nav2_monitor::ActionType::SUPERVISOR);
  EXPECT_EQ(faults[0].level, nav2_monitor::FaultLevel::ERROR);

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, ChassisAllMissingTimeoutTriggersIdleWarning)
{
  const std::string config_text = R"(
chassis_stationary:
  enabled: 1
  imu_topic: ""
  source_timeout_s: 10.0
  idle_timeout_s: 0.0
  command_speed_threshold: 0.05
  moto_speed_threshold: 0.05
  odom_speed_threshold: 0.03
  anomaly_level: "ERROR"
  idle_level: "WARNING"
  anomaly_actions: ["supervisor"]
  idle_actions: ["none"]
modules:
  - name: "dummy"
    supervisor: 1
    safety_system: 0
)";
  const std::string config_path = write_temp_config(config_text, "chassis_all_missing_timeout");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_chassis_all_missing_timeout");
  nav2_monitor::FaultDetector detector(node.get());
  detector.load_config(config_path);

  auto faults_first = detector.detect_faults();
  EXPECT_TRUE(faults_first.empty());

  auto faults_second = detector.detect_faults();
  EXPECT_TRUE(faults_second.empty());

  auto faults_third = detector.detect_faults();
  ASSERT_EQ(faults_third.size(), 1u);
  EXPECT_EQ(faults_third[0].module_name, "chassis_stationary");
  EXPECT_EQ(faults_third[0].action, nav2_monitor::ActionType::NONE);
  EXPECT_EQ(faults_third[0].level, nav2_monitor::FaultLevel::WARNING);

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, ChassisCommandAndMotoBothHasNoFault)
{
  const std::string config_text = R"(
chassis_stationary:
  enabled: 1
  imu_topic: ""
  source_timeout_s: 10.0
  idle_timeout_s: 30.0
  command_speed_threshold: 0.05
  moto_speed_threshold: 0.05
  odom_speed_threshold: 0.03
  anomaly_level: "ERROR"
  idle_level: "WARNING"
  anomaly_actions: ["supervisor"]
  idle_actions: ["none"]
modules:
  - name: "dummy"
    supervisor: 1
    safety_system: 0
)";
  const std::string config_path = write_temp_config(config_text, "chassis_both_has");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_chassis_both_has");
  nav2_monitor::FaultDetector detector(node.get());
  detector.load_config(config_path);
  detector.update_command_speed(0.5, node->now());
  detector.update_moto_speed(0.2, 0.25, node->now(), true);

  auto faults = detector.detect_faults();
  EXPECT_TRUE(faults.empty());

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, ChassisImuTruthSuppressesFalseStuckWhenDriveRequestExists)
{
  const std::string config_text = R"(
chassis_stationary:
  enabled: 1
  odom_topic: ""
  imu_topic: "/livox/imu"
  source_timeout_s: 10.0
  idle_timeout_s: 30.0
  command_speed_threshold: 0.05
  moto_speed_threshold: 0.05
  odom_speed_threshold: 0.03
  imu_speed_threshold: 0.03
  imu_yaw_rate_threshold: 0.08
  anomaly_level: "ERROR"
  idle_level: "WARNING"
  anomaly_actions: ["supervisor"]
  idle_actions: ["none"]
modules:
  - name: "dummy"
    supervisor: 1
    safety_system: 0
)";
  const std::string config_path = write_temp_config(config_text, "chassis_imu_truth");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_chassis_imu_truth");
  nav2_monitor::FaultDetector detector(node.get());
  nav2_monitor::MonitorDataStore store;
  detector.load_config(config_path);

  const auto now = node->now();
  store.set_command_speed(0.5, now);
  store.set_imu_motion(0.2, 0.0, now);

  auto faults = detector.detect_faults(store, now);
  EXPECT_TRUE(faults.empty());

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, ChassisImuTruthDoesNotUseControlChainUnexpectedMotionRule)
{
  const std::string config_text = R"(
chassis_stationary:
  enabled: 1
  odom_topic: ""
  imu_topic: "/livox/imu"
  source_timeout_s: 10.0
  idle_timeout_s: 30.0
  command_speed_threshold: 0.05
  moto_speed_threshold: 0.05
  odom_speed_threshold: 0.03
  imu_speed_threshold: 0.03
  imu_yaw_rate_threshold: 0.08
  anomaly_level: "ERROR"
  idle_level: "WARNING"
  anomaly_actions: ["supervisor"]
  idle_actions: ["none"]
modules:
  - name: "dummy"
    supervisor: 1
    safety_system: 0
)";
  const std::string config_path = write_temp_config(config_text, "chassis_imu_unexpected_motion");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_chassis_imu_unexpected_motion");
  nav2_monitor::FaultDetector detector(node.get());
  nav2_monitor::MonitorDataStore store;
  detector.load_config(config_path);

  const auto now = node->now();
  store.set_imu_motion(0.2, 0.0, now);

  auto faults_first = detector.detect_faults(store, now);
  EXPECT_TRUE(faults_first.empty());

  auto faults = detector.detect_faults(store, now);
  EXPECT_TRUE(faults.empty());

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, ModuleSafetySystemSlowDownCarriesConfiguredPercentage)
{
  const std::string config_text = R"(
modules:
  - name: "navigation"
    supervisor: 0
    safety_system: 1
    safety_slow_down_percentage: 35.0
    nodes:
      - "controller_server"
)";
  const std::string config_path = write_temp_config(config_text, "module_safety_slow_down");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_module_safety_slow_down");
  nav2_monitor::FaultDetector detector(node.get());
  detector.load_config(config_path);

  detector.update_node_status({{"controller_server", false}});
  detector.update_topic_freq({});

  auto faults = detector.detect_faults();

  ASSERT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].action, nav2_monitor::ActionType::SAFETY_SYSTEM);
  EXPECT_EQ(faults[0].safety_command, nav2_monitor::SafetyCommandType::SLOW_DOWN);
  EXPECT_DOUBLE_EQ(faults[0].safety_slow_down_percentage, 35.0);

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, FeedbackRuleSafetySystemOverrideUsesRuleSettings)
{
  const std::string config_text = R"(
modules:
  - name: "navigation"
    supervisor: 1
    safety_system: 2
    nodes:
      - "controller_server"
    feedback_rules:
      - source_topic: "/controller/feedback"
        metric_name: "tracking_error"
        min_value: 0.0
        max_value: 0.5
        max_stale_s: 2.0
        level: "CRITICAL"
        safety_system: 1
        safety_slow_down_percentage: 25.0
        actions: ["safety_system", "supervisor"]
)";
  const std::string config_path = write_temp_config(config_text, "feedback_rule_safety_override");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_feedback_rule_safety_override");
  nav2_monitor::FaultDetector detector(node.get());
  detector.set_feedback_default_max_stale(2.0);
  detector.load_config(config_path);

  detector.update_node_status({{"controller_server", true}});
  detector.update_topic_freq({});
  detector.update_feedback_sample(
    "navigation", "/controller/feedback", "tracking_error", 0.9, true, node->now());

  auto faults_first = detector.detect_faults();
  EXPECT_TRUE(faults_first.empty());

  auto faults = detector.detect_faults();
  ASSERT_EQ(faults.size(), 2u);
  EXPECT_EQ(faults[0].action, nav2_monitor::ActionType::SAFETY_SYSTEM);
  EXPECT_EQ(faults[0].safety_command, nav2_monitor::SafetyCommandType::SLOW_DOWN);
  EXPECT_DOUBLE_EQ(faults[0].safety_slow_down_percentage, 25.0);
  EXPECT_EQ(faults[1].action, nav2_monitor::ActionType::SUPERVISOR);

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, FeedbackRuleSafetySystemZeroDisablesSafetyFault)
{
  const std::string config_text = R"(
modules:
  - name: "navigation"
    supervisor: 1
    safety_system: 3
    nodes:
      - "controller_server"
    feedback_rules:
      - source_topic: "/controller/feedback"
        metric_name: "tracking_error"
        min_value: 0.0
        max_value: 0.5
        max_stale_s: 2.0
        level: "CRITICAL"
        safety_system: 0
        actions: ["safety_system", "supervisor"]
)";
  const std::string config_path = write_temp_config(config_text, "feedback_rule_disable_safety");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_feedback_rule_disable_safety");
  nav2_monitor::FaultDetector detector(node.get());
  detector.set_feedback_default_max_stale(2.0);
  detector.load_config(config_path);

  detector.update_node_status({{"controller_server", true}});
  detector.update_topic_freq({});
  detector.update_feedback_sample(
    "navigation", "/controller/feedback", "tracking_error", 0.9, true, node->now());

  auto faults_first = detector.detect_faults();
  EXPECT_TRUE(faults_first.empty());

  auto faults = detector.detect_faults();
  ASSERT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].action, nav2_monitor::ActionType::SUPERVISOR);

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, ChassisSafetySystemUsesConfiguredEmergencyStop)
{
  const std::string config_text = R"(
chassis_stationary:
  enabled: 1
  imu_topic: ""
  source_timeout_s: 10.0
  idle_timeout_s: 30.0
  command_speed_threshold: 0.05
  moto_speed_threshold: 0.05
  odom_speed_threshold: 0.03
  anomaly_level: "ERROR"
  idle_level: "WARNING"
  safety_system: 3
  anomaly_actions: ["safety_system"]
  idle_actions: ["none"]
modules:
  - name: "dummy"
    supervisor: 0
    safety_system: 0
)";
  const std::string config_path = write_temp_config(config_text, "chassis_safety_emergency_stop");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_chassis_safety_emergency_stop");
  nav2_monitor::FaultDetector detector(node.get());
  detector.load_config(config_path);
  detector.update_command_speed(0.5, node->now());
  detector.update_odom_speed(0.0, node->now());

  auto faults_first = detector.detect_faults();
  EXPECT_TRUE(faults_first.empty());

  auto faults = detector.detect_faults();
  ASSERT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].module_name, "chassis_stationary");
  EXPECT_EQ(faults[0].action, nav2_monitor::ActionType::SAFETY_SYSTEM);
  EXPECT_EQ(faults[0].safety_command, nav2_monitor::SafetyCommandType::EMERGENCY_STOP);
  EXPECT_DOUBLE_EQ(faults[0].safety_slow_down_percentage, 50.0);

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, StoreBackedWatchTopicDetectionWorks)
{
  const std::string config_text = R"(
modules:
  - name: "navigation"
    supervisor: 1
    safety_system: 0
    nodes:
      - "controller_server"
    watch_topics:
      - name: "/cmd_vel"
        min_hz: 5.0
)";
  const std::string config_path = write_temp_config(config_text, "store_watch_topic");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_store_watch_topic");
  nav2_monitor::FaultDetector detector(node.get());
  nav2_monitor::MonitorDataStore store;
  detector.load_config(config_path);

  const auto now = node->now();
  store.mark_node_seen("controller_server", now);
  store.set_watch_topic_publisher("/cmd_vel", true);

  auto faults_first = detector.detect_faults(store, now);
  EXPECT_TRUE(faults_first.empty());

  auto faults = detector.detect_faults(store, now);
  ASSERT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].action, nav2_monitor::ActionType::SUPERVISOR);
  EXPECT_NE(faults[0].fault_key.find("topic_legacy:/cmd_vel"), std::string::npos);

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, StoreBackedFeedbackDetectionWorks)
{
  const std::string config_text = R"(
modules:
  - name: "navigation"
    supervisor: 1
    safety_system: 1
    nodes:
      - "controller_server"
    feedback_rules:
      - source_topic: "/controller/feedback"
        metric_name: "tracking_error"
        min_value: 0.0
        max_value: 0.5
        max_stale_s: 2.0
        level: "CRITICAL"
        actions: ["safety_system", "supervisor"]
)";
  const std::string config_path = write_temp_config(config_text, "store_feedback");

  auto node = std::make_shared<rclcpp::Node>("fault_detector_test_store_feedback");
  nav2_monitor::FaultDetector detector(node.get());
  nav2_monitor::MonitorDataStore store;
  detector.set_feedback_default_max_stale(2.0);
  detector.load_config(config_path);

  const auto now = node->now();
  store.mark_node_seen("controller_server", now);
  store.add_feedback_sample(
    "navigation", "/controller/feedback", "tracking_error", 0.9, true, now);

  auto faults_first = detector.detect_faults(store, now);
  EXPECT_TRUE(faults_first.empty());

  auto faults = detector.detect_faults(store, now);
  ASSERT_EQ(faults.size(), 2u);
  EXPECT_NE(faults[0].fault_key.find("feedback:/controller/feedback:tracking_error"), std::string::npos);

  std::remove(config_path.c_str());
}

TEST_F(FaultDetectorTest, FaultStateCoordinatorDistinguishesTriggerAndRecoverEdges)
{
  nav2_monitor::FaultStateCoordinator coordinator;

  nav2_monitor::FaultInfo safety_fault;
  safety_fault.fault_key = "navigation|node_inactive|action=2";
  safety_fault.module_name = "navigation";
  safety_fault.level = nav2_monitor::FaultLevel::CRITICAL;
  safety_fault.reason = "Node inactive";
  safety_fault.action = nav2_monitor::ActionType::SAFETY_SYSTEM;
  safety_fault.safety_command = nav2_monitor::SafetyCommandType::EMERGENCY_STOP;
  safety_fault.safety_slow_down_percentage = 0.0;

  auto trigger_update = coordinator.update({safety_fault});
  ASSERT_EQ(trigger_update.edge_events.size(), 1u);
  EXPECT_EQ(trigger_update.edge_events[0].edge, nav2_monitor::FaultEdgeType::TRIGGER);
  ASSERT_TRUE(trigger_update.safety_update.has_value());
  EXPECT_TRUE(trigger_update.safety_update->active);
  EXPECT_EQ(trigger_update.safety_update->command, nav2_monitor::SafetyCommandType::EMERGENCY_STOP);

  auto steady_update = coordinator.update({safety_fault});
  EXPECT_TRUE(steady_update.edge_events.empty());
  EXPECT_FALSE(steady_update.safety_update.has_value());

  auto recover_update = coordinator.update({});
  ASSERT_EQ(recover_update.edge_events.size(), 1u);
  EXPECT_EQ(recover_update.edge_events[0].edge, nav2_monitor::FaultEdgeType::RECOVER);
  EXPECT_NE(
    recover_update.edge_events[0].fault.reason.find(
      "RECOVER fault_key=navigation|node_inactive|action=2; previous_reason=Node inactive"),
    std::string::npos);
  ASSERT_TRUE(recover_update.safety_update.has_value());
  EXPECT_FALSE(recover_update.safety_update->active);
}

TEST_F(FaultDetectorTest, FaultStateCoordinatorChoosesMostStrictSafetyCommand)
{
  nav2_monitor::FaultStateCoordinator coordinator;

  nav2_monitor::FaultInfo slow_fault;
  slow_fault.fault_key = "navigation|feedback|metric_a|action=2";
  slow_fault.module_name = "navigation";
  slow_fault.level = nav2_monitor::FaultLevel::ERROR;
  slow_fault.reason = "Need slow down";
  slow_fault.action = nav2_monitor::ActionType::SAFETY_SYSTEM;
  slow_fault.safety_command = nav2_monitor::SafetyCommandType::SLOW_DOWN;
  slow_fault.safety_slow_down_percentage = 40.0;

  nav2_monitor::FaultInfo stop_fault = slow_fault;
  stop_fault.fault_key = "navigation|feedback|metric_b|action=2";
  stop_fault.reason = "Need soft stop";
  stop_fault.safety_command = nav2_monitor::SafetyCommandType::SOFT_STOP;
  stop_fault.safety_slow_down_percentage = 0.0;

  auto update = coordinator.update({slow_fault, stop_fault});
  ASSERT_TRUE(update.safety_update.has_value());
  EXPECT_TRUE(update.safety_update->active);
  EXPECT_EQ(update.safety_update->command, nav2_monitor::SafetyCommandType::SOFT_STOP);

  auto downgrade_update = coordinator.update({slow_fault});
  ASSERT_TRUE(downgrade_update.safety_update.has_value());
  EXPECT_TRUE(downgrade_update.safety_update->active);
  EXPECT_EQ(downgrade_update.safety_update->command, nav2_monitor::SafetyCommandType::SLOW_DOWN);
  EXPECT_DOUBLE_EQ(downgrade_update.safety_update->slow_down_percentage, 40.0);

  auto resume_update = coordinator.update({});
  ASSERT_TRUE(resume_update.safety_update.has_value());
  EXPECT_FALSE(resume_update.safety_update->active);
}

}  // namespace
