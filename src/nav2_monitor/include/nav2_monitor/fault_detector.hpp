#ifndef NAV2_MONITOR__FAULT_DETECTOR_HPP_
#define NAV2_MONITOR__FAULT_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <string>
#include <map>
#include <vector>

namespace nav2_monitor
{

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
  SUPERVISOR = 1,      // 软重启
  SAFETY_SYSTEM = 2    // 应急措施
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
  std::map<std::string, double> topic_min_hz;
  struct FeedbackRule
  {
    std::string topic_name;
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

struct ChassisStationaryConfig
{
  bool enabled;
  std::string module_name;
  std::string command_topic;
  std::string moto_topic;
  std::string odom_topic;
  double source_timeout_s;
  double idle_timeout_s;
  double command_speed_threshold;
  double moto_speed_threshold;
  double odom_speed_threshold;
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
  FaultDetector(rclcpp::Node* node);

  void load_config(const std::string& config_file);

  // 从监控模块获取数据
  void update_node_status(const std::map<std::string, bool>& node_active);
  void update_topic_freq(const std::map<std::string, double>& topic_freq);
  void update_feedback_sample(
    const std::string & module_name, const std::string & topic_name,
    const std::string & metric_name, double value, bool valid, const rclcpp::Time & stamp);
  void set_feedback_default_max_stale(double default_max_stale_s);
  void update_command_speed(double speed, const rclcpp::Time & stamp);
  void update_moto_speed(double left_speed_rad, double right_speed_rad, const rclcpp::Time & stamp, bool valid);
  void update_odom_speed(double linear_speed, const rclcpp::Time & stamp);
  bool chassis_stationary_enabled() const;
  const ChassisStationaryConfig & get_chassis_stationary_config() const;
  bool has_module_configs() const;
  const std::vector<std::string> & get_monitored_nodes() const;
  const std::vector<std::string> & get_monitored_topics() const;
  const MultiValueJudgeConfig & get_multi_value_judge_config() const;

  // 判断逻辑
  std::vector<FaultInfo> detect_faults();

private:
  struct FeedbackState
  {
    double last_value;
    bool last_valid;
    bool received;
    rclcpp::Time first_seen;
    rclcpp::Time last_seen;
    std::deque<rclcpp::Time> msg_times;
  };

  struct ChassisState
  {
    bool command_received;
    double command_speed;
    rclcpp::Time command_stamp;
    bool moto_received;
    bool moto_valid;
    double left_speed_rad;
    double right_speed_rad;
    rclcpp::Time moto_stamp;
    bool odom_received;
    double odom_speed;
    rclcpp::Time odom_stamp;
    bool idle_tracking;
    rclcpp::Time idle_start_time;
  };

  struct RuleJudgeState
  {
    size_t abnormal_count{0};
    size_t normal_count{0};
    bool latched{false};
    std::string last_reason;
  };

  rclcpp::Node* node_;
  std::vector<ModuleConfig> modules_;
  ChassisStationaryConfig chassis_cfg_;
  MultiValueJudgeConfig multi_value_cfg_;
  std::vector<std::string> monitored_nodes_;
  std::vector<std::string> monitored_topics_;

  std::map<std::string, bool> node_status_;
  std::map<std::string, double> topic_freq_;
  std::map<std::string, FeedbackState> feedback_state_;
  std::map<std::string, RuleJudgeState> feedback_judge_state_;
  std::map<std::string, RuleJudgeState> topic_judge_state_;
  std::map<std::string, RuleJudgeState> chassis_judge_state_;
  ChassisState chassis_state_;
  rclcpp::Time config_loaded_time_;
  double feedback_default_max_stale_s_;

  bool check_module_nodes(const ModuleConfig& module);
  void append_feedback_faults(
    const ModuleConfig & module, const ModuleConfig::FeedbackRule & rule,
    const std::string & reason, std::vector<FaultInfo> & faults, const rclcpp::Time & now);
  void append_chassis_faults(
    const std::string & fault_key_prefix, FaultLevel level,
    const std::vector<ActionType> & actions, const std::string & reason,
    std::vector<FaultInfo> & faults, const rclcpp::Time & now);
  bool update_multi_value_state(
    const std::string & key, bool abnormal, const std::string & reason,
    std::map<std::string, RuleJudgeState> & states, std::string & active_reason);
  std::string feedback_key(
    const std::string & module_name, const std::string & topic_name, const std::string & metric_name) const;
};

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__FAULT_DETECTOR_HPP_
