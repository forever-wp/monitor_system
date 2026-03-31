#ifndef NAV2_MONITOR__FAULT_DETECTOR_HPP_
#define NAV2_MONITOR__FAULT_DETECTOR_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "nav2_monitor/monitor_data_store.hpp"

namespace nav2_monitor
{

class WatchTopicEvaluator;
class FeedbackRuleEvaluator;
class ChassisEvaluator;
class CollisionEvaluator;

enum class FaultLevel
{
  NORMAL = 0,
  WARNING = 1,
  ERROR = 2,
  CRITICAL = 3
};

enum class ActionType
{
  NONE = 0,
  SUPERVISOR = 1,
  SAFETY_SYSTEM = 2
};

enum class SafetyCommandType
{
  NONE = 0,
  SLOW_DOWN = 1,
  SOFT_STOP = 2,
  EMERGENCY_STOP = 3
};

struct ModuleConfig
{
  std::string name;
  bool enable_supervisor;
  SafetyCommandType safety_command;
  double safety_slow_down_percentage;
  std::vector<std::string> nodes;
  std::map<std::string, double> watch_topic_min_hz;

  struct FeedbackRule
  {
    std::string source_topic;
    std::string metric_name;
    bool has_min_value;
    bool has_max_value;
    double min_value;
    double max_value;
    double min_hz;
    double max_stale_s;
    FaultLevel level;
    bool has_safety_command_override;
    SafetyCommandType safety_command;
    bool has_safety_slow_down_percentage;
    double safety_slow_down_percentage;
    std::vector<ActionType> actions;
  };

  std::vector<FeedbackRule> feedback_rules;
};


enum class CollisionModelType
{
  ZONE = 0,
  APPROACH = 1
};

struct UltrasonicSensorConfig
{
  size_t index{0};
  bool enabled{true};
  double x{0.0};
  double y{0.0};
  double yaw_deg{0.0};
  double max_distance{1.5};
  double weight{1.0};
};

struct CollisionZoneConfig
{
  std::string name;
  CollisionModelType model{CollisionModelType::ZONE};
  std::vector<CollisionPoint> points;
  double min_points{1.0};
  FaultLevel level{FaultLevel::ERROR};
  SafetyCommandType safety_command{SafetyCommandType::SOFT_STOP};
  double safety_slow_down_percentage{50.0};
  std::vector<ActionType> actions;
  bool enabled{true};
  bool visualize{true};
  std::string polygon_pub_topic;
  double time_before_collision{1.0};
  double recover_time_before_collision{0.0};
  double min_hold_time_s{0.0};
  double simulation_time_step{0.1};
};

struct CollisionDetectionConfig
{
  bool enabled{false};
  std::string module_name{"collision_detection"};
  std::string scan_topic{"/scan"};
  std::string pointcloud_topic{""};
  std::string ultrasonic_topic{""};
  std::string prediction_speed_topic{"/cmd_vel"};
  std::string ultrasonic_distances_key{"distances"};
  std::string ultrasonic_scene_flag_key{"scene_flag"};
  double ultrasonic_blind_distance{0.2};
  double ultrasonic_out_of_range_value{1.0};
  double pointcloud_min_height{0.0};
  double pointcloud_max_height{2.0};
  double source_timeout_s{0.5};
  std::vector<CollisionPoint> footprint_points;
  std::vector<UltrasonicSensorConfig> ultrasonic_sensors;
  std::vector<CollisionZoneConfig> zones;
};

struct ChassisStationaryConfig
{
  bool enabled;
  std::string module_name;
  std::string command_topic;
  std::string moto_topic;
  std::string odom_topic;
  std::string imu_topic;
  double source_timeout_s;
  double idle_timeout_s;
  double command_speed_threshold;
  double moto_speed_threshold;
  double odom_speed_threshold;
  double imu_speed_threshold;
  double imu_yaw_rate_threshold;
  double imu_static_command_threshold;
  int imu_bias_calibration_samples;
  double imu_decay_rate;
  FaultLevel anomaly_level;
  FaultLevel idle_level;
  SafetyCommandType safety_command;
  double safety_slow_down_percentage;
  std::vector<ActionType> anomaly_actions;
  std::vector<ActionType> idle_actions;
};

struct MultiValueJudgeConfig
{
  size_t trigger_count;
  size_t recover_count;
};

struct FaultInfo
{
  std::string fault_key;
  std::string module_name;
  FaultLevel level;
  std::string reason;
  ActionType action;
  SafetyCommandType safety_command;
  double safety_slow_down_percentage;
  rclcpp::Time timestamp;
};

class FaultDetector
{
public:
  explicit FaultDetector(rclcpp::Node * node);
  ~FaultDetector();

  void load_config(const std::string & config_file);

  void update_node_status(const std::map<std::string, bool> & node_active);
  void update_topic_freq(const std::map<std::string, double> & topic_freq);
  void update_feedback_sample(
    const std::string & module_name, const std::string & topic_name,
    const std::string & metric_name, double value, bool valid, const rclcpp::Time & stamp);
  void set_feedback_default_max_stale(double default_max_stale_s);
  void update_command_speed(double speed, const rclcpp::Time & stamp);
  void update_moto_speed(
    double left_speed_rad, double right_speed_rad, const rclcpp::Time & stamp, bool valid);
  void update_odom_speed(double linear_speed, const rclcpp::Time & stamp);
  bool chassis_stationary_enabled() const;
  const ChassisStationaryConfig & get_chassis_stationary_config() const;
  bool has_module_configs() const;
  const std::vector<std::string> & get_monitored_nodes() const;
  const std::vector<std::string> & get_watched_topics() const;
  const std::vector<std::string> & get_monitored_transforms() const;
  bool is_watch_topic_frequency_required(const std::string & topic) const;
  const MultiValueJudgeConfig & get_multi_value_judge_config() const;
  bool collision_detection_enabled() const;
  const CollisionDetectionConfig & get_collision_detection_config() const;

  std::vector<FaultInfo> detect_faults();
  std::vector<FaultInfo> detect_faults(const MonitorDataStore & store, const rclcpp::Time & now);

private:
  bool check_module_nodes(
    const ModuleConfig & module,
    const MonitorDataStore & store,
    const rclcpp::Time & now) const;

  rclcpp::Node * node_;
  std::vector<ModuleConfig> modules_;
  CollisionDetectionConfig collision_cfg_;
  ChassisStationaryConfig chassis_cfg_;
  MultiValueJudgeConfig multi_value_cfg_;
  std::vector<std::string> monitored_nodes_;
  std::vector<std::string> watched_topics_;
  std::vector<std::string> monitored_transforms_;
  rclcpp::Time config_loaded_time_;
  double feedback_default_max_stale_s_;
  MonitorDataStore compatibility_store_;
  std::unique_ptr<WatchTopicEvaluator> watch_topic_evaluator_;
  std::unique_ptr<FeedbackRuleEvaluator> feedback_rule_evaluator_;
  std::unique_ptr<ChassisEvaluator> chassis_evaluator_;
  std::unique_ptr<CollisionEvaluator> collision_evaluator_;
};

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__FAULT_DETECTOR_HPP_
