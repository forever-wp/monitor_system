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
  const CollisionTtcVisualizationState & get_ttc_visualization() const;

private:
  struct RuleJudgeState
  {
    size_t abnormal_count{0};
    size_t normal_count{0};
    bool latched{false};
    rclcpp::Time latched_since{0, 0, RCL_ROS_TIME};
    std::string last_reason;
  };

  enum class RuntimeMotionDirection
  {
    UNKNOWN = 0,
    FORWARD = 1,
    REVERSE = 2
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
  static double point_to_polyline_distance(
    const CollisionPoint & point,
    const std::vector<CollisionPoint> & polyline);
  static double compute_bounding_radius(
    const std::vector<CollisionPoint> & footprint);
  static std::vector<CollisionPoint> transform_polygon(
    const std::vector<CollisionPoint> & polygon,
    double x,
    double y,
    double yaw);
  static std::vector<CollisionPoint> sample_centerline(
    double linear_x,
    double linear_y,
    double angular_z,
    double horizon_s,
    double time_step_s);
  static std::vector<CollisionPoint> collect_ttc_candidate_points(
    const std::vector<CollisionPoint> & points,
    const std::vector<CollisionPoint> & centerline,
    double corridor_radius);
  static std::vector<CollisionPoint> collect_evaluation_points(
    const CollisionDetectionConfig & cfg,
    const MonitorDataStore & store,
    const rclcpp::Time & now);
  static bool has_fresh_evaluation_source(
    const CollisionDetectionConfig & cfg,
    const MonitorDataStore & store,
    const rclcpp::Time & now,
    std::string & reason);
  static std::vector<CollisionPoint> downsample_candidate_points(
    const std::vector<CollisionPoint> & points,
    double resolution);
  static std::vector<CollisionPoint> build_corridor_outline(
    const std::vector<CollisionPoint> & centerline,
    double corridor_radius);
  static double estimate_trajectory_collision_time(
    const CollisionPoint & point,
    const std::vector<CollisionPoint> & footprint,
    double linear_x,
    double linear_y,
    double angular_z,
    double horizon_s,
    double time_step_s,
    double & min_clearance);
  static void sample_trajectory_visualization(
    const std::vector<CollisionPoint> & footprint,
    double linear_x,
    double linear_y,
    double angular_z,
    double horizon_s,
    double time_step_s,
    std::vector<CollisionPoint> & trajectory_points,
    std::vector<std::vector<CollisionPoint>> & footprint_samples);
  bool update_multi_value_state(
    const std::string & key,
    bool abnormal,
    const std::string & reason,
    const rclcpp::Time & now,
    double min_hold_time_s,
    std::string & active_reason) const;
  RuntimeMotionDirection resolve_runtime_motion_direction(
    const ChassisRuntimeState & chassis_state,
    double direction_speed_threshold,
    size_t direction_confirm_count,
    const rclcpp::Time & now,
    double source_timeout_s) const;
  static bool zone_matches_motion_direction(
    CollisionMotionDirectionType zone_direction,
    RuntimeMotionDirection runtime_direction);
  void append_zone_faults(
    const CollisionDetectionConfig & cfg,
    const CollisionZoneConfig & zone,
    const std::string & reason,
    std::vector<FaultInfo> & faults,
    const rclcpp::Time & now) const;
  void append_source_faults(
    const CollisionDetectionConfig & cfg,
    const std::string & reason,
    std::vector<FaultInfo> & faults,
    const rclcpp::Time & now) const;

  MultiValueJudgeConfig multi_value_cfg_{2, 2};
  mutable std::map<std::string, RuleJudgeState> judge_states_;
  mutable RuntimeMotionDirection stable_motion_direction_{RuntimeMotionDirection::UNKNOWN};
  mutable RuntimeMotionDirection pending_motion_direction_{RuntimeMotionDirection::UNKNOWN};
  mutable size_t pending_motion_direction_count_{0};
  mutable rclcpp::Time last_source_fallback_log_time_{0, 0, RCL_ROS_TIME};
  mutable CollisionTtcVisualizationState ttc_visualization_;
};

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__COLLISION_EVALUATOR_HPP_
