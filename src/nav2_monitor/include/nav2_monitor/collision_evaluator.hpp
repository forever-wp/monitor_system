#ifndef NAV2_MONITOR__COLLISION_EVALUATOR_HPP_
#define NAV2_MONITOR__COLLISION_EVALUATOR_HPP_

#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "nav2_monitor/fault_detector.hpp"
#include "nav2_monitor/monitor_data_store.hpp"

namespace nav2_monitor
{

class CollisionEvaluator
{
public:
  void set_multi_value_config(const MultiValueJudgeConfig & config);
  void reset();
  std::vector<FaultInfo> evaluate(
    const CollisionDetectionConfig & cfg,
    const MonitorDataStore & store,
    const rclcpp::Time & now) const;

private:
  struct RuleJudgeState
  {
    size_t abnormal_count{0};
    size_t normal_count{0};
    bool latched{false};
    rclcpp::Time latched_since{0, 0, RCL_ROS_TIME};
    std::string last_reason;
  };

  static bool is_point_inside_polygon(
    const CollisionPoint & point,
    const std::vector<CollisionPoint> & polygon);
  static double point_to_segment_distance(
    const CollisionPoint & point,
    const CollisionPoint & seg_start,
    const CollisionPoint & seg_end);
  static double point_to_polygon_distance(
    const CollisionPoint & point,
    const std::vector<CollisionPoint> & polygon);
  static std::vector<CollisionPoint> transform_polygon(
    const std::vector<CollisionPoint> & polygon,
    double x,
    double y,
    double yaw);
  static double estimate_trajectory_collision_time(
    const CollisionPoint & point,
    const std::vector<CollisionPoint> & footprint,
    double linear_x,
    double linear_y,
    double angular_z,
    double horizon_s,
    double time_step_s,
    double & min_clearance);
  bool update_multi_value_state(
    const std::string & key,
    bool abnormal,
    const std::string & reason,
    const rclcpp::Time & now,
    double min_hold_time_s,
    std::string & active_reason) const;
  void append_zone_faults(
    const CollisionDetectionConfig & cfg,
    const CollisionZoneConfig & zone,
    const std::string & reason,
    std::vector<FaultInfo> & faults,
    const rclcpp::Time & now) const;

  MultiValueJudgeConfig multi_value_cfg_{2, 2};
  mutable std::map<std::string, RuleJudgeState> judge_states_;
};

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__COLLISION_EVALUATOR_HPP_
