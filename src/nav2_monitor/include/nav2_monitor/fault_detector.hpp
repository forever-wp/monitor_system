#ifndef NAV2_MONITOR__FAULT_DETECTOR_HPP_
#define NAV2_MONITOR__FAULT_DETECTOR_HPP_

#include <limits>
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
  TTC = 1,
  APPROACH = TTC
};

enum class CollisionMotionDirectionType
{
  BOTH = 0,
  FORWARD = 1,
  REVERSE = 2
};

enum class AutoFootprintZoneType
{
  NONE = 0,
  FRONT_SLOW = 1,
  FRONT_STOP = 2
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
  CollisionMotionDirectionType motion_direction{CollisionMotionDirectionType::BOTH};
  AutoFootprintZoneType auto_footprint_zone{AutoFootprintZoneType::NONE};
  std::vector<CollisionPoint> points;
  std::vector<CollisionPoint> fast_points;
  std::vector<CollisionPoint> safe_points;
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
  double ttc_horizon_s{0.0};
  double corridor_margin{0.10};
  double candidate_downsample_resolution{0.08};
  double simulation_time_step{0.1};
};

struct CollisionDetectionConfig
{
  bool enabled{false};
  std::string module_name{"collision_detection"};
  std::string scan_topic{"/scan"};
  std::string pointcloud_topic{""};
  std::string voxel_topic{""};
  std::string ultrasonic_topic{""};
  std::string prediction_speed_topic{"/cmd_vel"};
  std::string control_source_state_topic{"/control_source_state"};
  std::string prediction_speed_navigation_topic;
  std::string prediction_speed_miniapp_topic{"/cmd_vel_miniapp"};
  std::string prediction_speed_remote_topic{"/cmd_vel_remote"};
  std::string prediction_speed_other_topic{"/cmd_vel_other"};
  std::string ultrasonic_distances_key{"distances"};
  std::string ultrasonic_scene_flag_key{"scene_flag"};
  bool ttc_visualization_enabled{false};
  double ultrasonic_blind_distance{0.2};
  double ultrasonic_out_of_range_value{1.0};
  double pointcloud_min_height{0.0};
  double pointcloud_max_height{2.0};
  double voxel_min_occupancy{0.0};
  double voxel_min_height{-std::numeric_limits<double>::infinity()};
  double voxel_max_height{std::numeric_limits<double>::infinity()};
  double source_timeout_s{0.5};
  double direction_speed_threshold{0.05};
  size_t direction_confirm_count{3};
  bool navigation_safe_mode_active{false};
  bool navigation_mode_switch_enabled{false};
  std::string navigation_mode_topic{"/navigation_mode"};
  std::string navigation_fast_mode{"FAST"};
  std::string navigation_safe_mode{"SAFE"};
  double navigation_safe_enter_duration_s{0.15};
  double navigation_safe_clear_duration_s{1.0};
  double navigation_safe_min_hold_s{1.5};
  double navigation_mode_publish_cooldown_s{0.5};
  bool auto_footprint_zones_enabled{false};
  FaultLevel source_level{FaultLevel::ERROR};
  std::vector<ActionType> source_actions{ActionType::SUPERVISOR};
  std::vector<CollisionPoint> footprint_points;
  std::vector<UltrasonicSensorConfig> ultrasonic_sensors;
  std::vector<CollisionZoneConfig> zones;
};

struct CollisionTtcVisualizationState
{
  bool enabled{false};
  bool active{false};
  std::string zone_name;
  double ttc_s{-1.0};
  double threshold_s{0.0};
  double min_clearance{-1.0};
  CollisionPoint collision_point;
  std::vector<CollisionPoint> corridor_outline;
  std::vector<CollisionPoint> trajectory_points;
  std::vector<std::vector<CollisionPoint>> footprint_samples;
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
  double coast_grace_s;
  double command_speed_threshold;
  double moto_speed_threshold;
  double odom_speed_threshold;
  double imu_speed_threshold;
  double imu_yaw_rate_threshold;
  double imu_static_command_threshold;
  int imu_bias_calibration_samples;
  double imu_decay_rate;
  FaultLevel source_level;
  FaultLevel anomaly_level;
  FaultLevel idle_level;
  SafetyCommandType safety_command;
  double safety_slow_down_percentage;
  std::vector<ActionType> source_actions;
  std::vector<ActionType> anomaly_actions;
  std::vector<ActionType> idle_actions;
};

struct MultiValueJudgeConfig
{
  size_t trigger_count;
  size_t recover_count;
};

struct CombinedFaultRuleConfig
{
  std::string name;
  std::vector<std::string> when_all_fault_keys;
  FaultLevel level{FaultLevel::ERROR};
  std::vector<ActionType> actions;
  SafetyCommandType safety_command{SafetyCommandType::NONE};
  double safety_slow_down_percentage{0.0};
  std::string reason;
};

struct FaultInfo
{
  std::string fault_key;
  std::string module_name;
  FaultLevel level;
  std::string reason;
  std::string fault_type;
  std::string fault_model;
  std::string fault_name;
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
  bool vehicle_state_judge_enabled() const;
  const ChassisStationaryConfig & get_vehicle_state_judge_config() const;
  bool chassis_stationary_enabled() const;
  const ChassisStationaryConfig & get_chassis_stationary_config() const;
  bool has_module_configs() const;
  const std::vector<std::string> & get_monitored_nodes() const;
  const std::vector<std::string> & get_watched_topics() const;
  const std::vector<std::string> & get_monitored_transforms() const;
  bool is_watch_topic_frequency_required(const std::string & topic) const;
  double get_watch_topic_min_hz(const std::string & topic) const;
  const MultiValueJudgeConfig & get_multi_value_judge_config() const;
  bool collision_detection_enabled() const;
  const CollisionDetectionConfig & get_collision_detection_config() const;
  const CollisionTtcVisualizationState & get_collision_ttc_visualization() const;
  void set_collision_navigation_safe_mode(bool safe_mode_active);

  std::vector<FaultInfo> detect_faults();
  std::vector<FaultInfo> detect_faults(const MonitorDataStore & store, const rclcpp::Time & now);

private:
  bool check_module_nodes(
    const ModuleConfig & module,
    const MonitorDataStore & store,
    const rclcpp::Time & now) const;
  void append_combined_faults(
    const std::vector<FaultInfo> & base_faults,
    const rclcpp::Time & now,
    std::vector<FaultInfo> & faults) const;

  rclcpp::Node * node_;
  std::vector<ModuleConfig> modules_;
  CollisionDetectionConfig collision_cfg_;
  ChassisStationaryConfig chassis_cfg_;
  MultiValueJudgeConfig multi_value_cfg_;
  std::vector<CombinedFaultRuleConfig> combined_fault_rules_;
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
