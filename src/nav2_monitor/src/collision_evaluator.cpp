#include "nav2_monitor/collision_evaluator.hpp"

#include <sstream>
#include <cmath>
#include <limits>
#include <utility>

namespace nav2_monitor
{

static double point_norm(const CollisionPoint & point)
{
  return std::sqrt(point.x * point.x + point.y * point.y);
}


void CollisionEvaluator::set_multi_value_config(const MultiValueJudgeConfig & config)
{
  multi_value_cfg_ = config;
}

void CollisionEvaluator::reset()
{
  judge_states_.clear();
  last_motion_direction_ = RuntimeMotionDirection::UNKNOWN;
}

bool CollisionEvaluator::is_point_inside_polygon(
  const CollisionPoint & point,
  const std::vector<CollisionPoint> & polygon)
{
  if (polygon.size() < 3) {
    return false;
  }

  bool inside = false;
  size_t j = polygon.size() - 1;
  for (size_t i = 0; i < polygon.size(); j = i++) {
    const auto & pi = polygon[i];
    const auto & pj = polygon[j];
    const bool intersect =
      ((pi.y > point.y) != (pj.y > point.y)) &&
      (point.x < (pj.x - pi.x) * (point.y - pi.y) / ((pj.y - pi.y) + 1e-9) + pi.x);
    if (intersect) {
      inside = !inside;
    }
  }
  return inside;
}

double CollisionEvaluator::point_to_segment_distance(
  const CollisionPoint & point,
  const CollisionPoint & seg_start,
  const CollisionPoint & seg_end)
{
  const double dx = seg_end.x - seg_start.x;
  const double dy = seg_end.y - seg_start.y;
  const double length_sq = dx * dx + dy * dy;
  if (length_sq <= 1e-12) {
    const double px = point.x - seg_start.x;
    const double py = point.y - seg_start.y;
    return std::sqrt(px * px + py * py);
  }

  const double projection =
    ((point.x - seg_start.x) * dx + (point.y - seg_start.y) * dy) / length_sq;
  const double clamped = std::clamp(projection, 0.0, 1.0);
  const double nearest_x = seg_start.x + clamped * dx;
  const double nearest_y = seg_start.y + clamped * dy;
  const double diff_x = point.x - nearest_x;
  const double diff_y = point.y - nearest_y;
  return std::sqrt(diff_x * diff_x + diff_y * diff_y);
}


double CollisionEvaluator::point_to_polygon_distance(
  const CollisionPoint & point,
  const std::vector<CollisionPoint> & polygon)
{
  if (polygon.size() < 3) {
    return point_norm(point);
  }

  if (is_point_inside_polygon(point, polygon)) {
    return 0.0;
  }

  double min_distance = std::numeric_limits<double>::infinity();
  for (size_t idx = 0; idx < polygon.size(); ++idx) {
    const auto & start = polygon[idx];
    const auto & end = polygon[(idx + 1) % polygon.size()];
    min_distance = std::min(min_distance, point_to_segment_distance(point, start, end));
  }
  return std::isfinite(min_distance) ? min_distance : point_norm(point);
}


std::vector<CollisionPoint> CollisionEvaluator::transform_polygon(
  const std::vector<CollisionPoint> & polygon,
  double x,
  double y,
  double yaw)
{
  std::vector<CollisionPoint> transformed;
  transformed.reserve(polygon.size());
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  for (const auto & point : polygon) {
    transformed.push_back(CollisionPoint{
      x + cos_yaw * point.x - sin_yaw * point.y,
      y + sin_yaw * point.x + cos_yaw * point.y,
      point.weight});
  }
  return transformed;
}


double CollisionEvaluator::estimate_trajectory_collision_time(
  const CollisionPoint & point,
  const std::vector<CollisionPoint> & footprint,
  double linear_x,
  double linear_y,
  double angular_z,
  double horizon_s,
  double time_step_s,
  double & min_clearance)
{
  min_clearance = std::numeric_limits<double>::infinity();
  double pos_x = 0.0;
  double pos_y = 0.0;
  double yaw = 0.0;
  const double dt = std::max(0.01, time_step_s);

  for (double sim_t = 0.0; sim_t <= horizon_s + 1e-9; sim_t += dt) {
    const auto oriented_footprint = transform_polygon(footprint, pos_x, pos_y, yaw);
    const double clearance = point_to_polygon_distance(point, oriented_footprint);
    min_clearance = std::min(min_clearance, clearance);
    if (clearance <= 1e-6) {
      return sim_t;
    }

    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    pos_x += (linear_x * cos_yaw - linear_y * sin_yaw) * dt;
    pos_y += (linear_x * sin_yaw + linear_y * cos_yaw) * dt;
    yaw += angular_z * dt;
  }

  return std::numeric_limits<double>::infinity();
}

void CollisionEvaluator::sample_trajectory_visualization(
  const std::vector<CollisionPoint> & footprint,
  double linear_x,
  double linear_y,
  double angular_z,
  double horizon_s,
  double time_step_s,
  std::vector<CollisionPoint> & trajectory_points,
  std::vector<std::vector<CollisionPoint>> & footprint_samples)
{
  trajectory_points.clear();
  footprint_samples.clear();

  double pos_x = 0.0;
  double pos_y = 0.0;
  double yaw = 0.0;
  const double dt = std::max(0.01, time_step_s);

  for (double sim_t = 0.0; sim_t <= horizon_s + 1e-9; sim_t += dt) {
    trajectory_points.push_back(CollisionPoint{pos_x, pos_y, 1.0});
    if (!footprint.empty()) {
      footprint_samples.push_back(transform_polygon(footprint, pos_x, pos_y, yaw));
    }

    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    pos_x += (linear_x * cos_yaw - linear_y * sin_yaw) * dt;
    pos_y += (linear_x * sin_yaw + linear_y * cos_yaw) * dt;
    yaw += angular_z * dt;
  }
}


bool CollisionEvaluator::update_multi_value_state(
  const std::string & key,
  bool abnormal,
  const std::string & reason,
  const rclcpp::Time & now,
  double min_hold_time_s,
  std::string & active_reason) const
{
  auto & state = judge_states_[key];
  if (abnormal) {
    state.abnormal_count++;
    state.normal_count = 0;
    if (!reason.empty()) {
      state.last_reason = reason;
    }
    if (!state.latched && state.abnormal_count >= multi_value_cfg_.trigger_count) {
      state.latched = true;
      state.latched_since = now;
    }
  } else {
    const bool hold_active = state.latched && min_hold_time_s > 0.0 &&
      (now - state.latched_since).seconds() < min_hold_time_s;
    if (hold_active) {
      state.normal_count = 0;
    } else {
      state.normal_count++;
      state.abnormal_count = 0;
      if (state.latched && state.normal_count >= multi_value_cfg_.recover_count) {
        state.latched = false;
        state.latched_since = rclcpp::Time(0, 0, RCL_ROS_TIME);
        state.last_reason.clear();
      }
    }
  }

  if (!state.latched) {
    return false;
  }

  active_reason = abnormal ? reason : state.last_reason;
  if (active_reason.empty()) {
    active_reason = "Fault latched";
  }
  return true;
}

CollisionEvaluator::RuntimeMotionDirection CollisionEvaluator::resolve_runtime_motion_direction(
  const ChassisRuntimeState & chassis_state,
  double direction_speed_threshold,
  const rclcpp::Time & now,
  double source_timeout_s) const
{
  const bool prediction_motion_fresh = chassis_state.prediction_speed_received &&
    (now - chassis_state.prediction_speed_stamp).seconds() <= source_timeout_s;

  if (prediction_motion_fresh) {
    if (chassis_state.prediction_linear_x > direction_speed_threshold) {
      last_motion_direction_ = RuntimeMotionDirection::FORWARD;
      return last_motion_direction_;
    }
    if (chassis_state.prediction_linear_x < -direction_speed_threshold) {
      last_motion_direction_ = RuntimeMotionDirection::REVERSE;
      return last_motion_direction_;
    }
  }

  return last_motion_direction_;
}

bool CollisionEvaluator::zone_matches_motion_direction(
  CollisionMotionDirectionType zone_direction,
  RuntimeMotionDirection runtime_direction)
{
  if (zone_direction == CollisionMotionDirectionType::BOTH ||
    runtime_direction == RuntimeMotionDirection::UNKNOWN)
  {
    return true;
  }

  if (zone_direction == CollisionMotionDirectionType::FORWARD) {
    return runtime_direction == RuntimeMotionDirection::FORWARD;
  }

  if (zone_direction == CollisionMotionDirectionType::REVERSE) {
    return runtime_direction == RuntimeMotionDirection::REVERSE;
  }

  return true;
}

const CollisionTtcVisualizationState & CollisionEvaluator::get_ttc_visualization() const
{
  return ttc_visualization_;
}

void CollisionEvaluator::append_zone_faults(
  const CollisionDetectionConfig & cfg,
  const CollisionZoneConfig & zone,
  const std::string & reason,
  std::vector<FaultInfo> & faults,
  const rclcpp::Time & now) const
{
  for (const auto & action : zone.actions) {
    if (action == ActionType::SAFETY_SYSTEM && zone.safety_command == SafetyCommandType::NONE) {
      continue;
    }

    FaultInfo fault;
    fault.fault_key = cfg.module_name + "|collision:" + zone.name + "|action=" +
      std::to_string(static_cast<int>(action));
    fault.module_name = cfg.module_name;
    fault.level = zone.level;
    fault.reason = reason;
    fault.action = action;
    fault.safety_command =
      action == ActionType::SAFETY_SYSTEM ? zone.safety_command : SafetyCommandType::NONE;
    fault.safety_slow_down_percentage =
      action == ActionType::SAFETY_SYSTEM ? zone.safety_slow_down_percentage : 0.0;
    fault.timestamp = now;
    faults.push_back(std::move(fault));
  }
}

std::vector<FaultInfo> CollisionEvaluator::evaluate(
  const CollisionDetectionConfig & cfg,
  const MonitorDataStore & store,
  const rclcpp::Time & now) const
{
  std::vector<FaultInfo> faults;
  ttc_visualization_.enabled = cfg.ttc_visualization_enabled;
  ttc_visualization_.active = false;
  ttc_visualization_.zone_name.clear();
  ttc_visualization_.ttc_s = -1.0;
  ttc_visualization_.threshold_s = 0.0;
  ttc_visualization_.min_clearance = -1.0;
  ttc_visualization_.collision_point = CollisionPoint{};
  ttc_visualization_.trajectory_points.clear();
  ttc_visualization_.footprint_samples.clear();
  if (!cfg.enabled) {
    return faults;
  }

  const auto points = store.get_collision_points(now, cfg.source_timeout_s);
  if (points.empty()) {
    return faults;
  }

  const auto & chassis_state = store.get_chassis_state();
  const bool prediction_speed_fresh = chassis_state.prediction_speed_received &&
    (now - chassis_state.prediction_speed_stamp).seconds() <= cfg.source_timeout_s;
  const double current_speed = prediction_speed_fresh ?
    std::fabs(chassis_state.prediction_speed) : 0.0;
  const auto runtime_direction = resolve_runtime_motion_direction(
    chassis_state, cfg.direction_speed_threshold, now, cfg.source_timeout_s);

  for (const auto & zone : cfg.zones) {
    if (!zone.enabled || zone.points.size() < 3) {
      continue;
    }

    const std::string key = cfg.module_name + "|collision:" + zone.name;
    const bool currently_latched = judge_states_[key].latched;
    bool abnormal = false;
    std::string reason;
    if (zone.model == CollisionModelType::APPROACH) {
      if (current_speed > 1e-3) {
        double min_collision_time = std::numeric_limits<double>::infinity();
        double min_clearance = std::numeric_limits<double>::infinity();
        CollisionPoint best_point;
        const bool use_trajectory_ttc = !cfg.footprint_points.empty();
        const double active_threshold =
          (currently_latched && zone.recover_time_before_collision > zone.time_before_collision) ?
          zone.recover_time_before_collision : zone.time_before_collision;
        for (const auto & point : points) {
          if (!is_point_inside_polygon(point, zone.points)) {
            continue;
          }

          double collision_time = std::numeric_limits<double>::infinity();
          double clearance = point_norm(point);
          if (use_trajectory_ttc) {
            collision_time = estimate_trajectory_collision_time(
              point,
              cfg.footprint_points,
              chassis_state.prediction_linear_x,
              chassis_state.prediction_linear_y,
              chassis_state.prediction_angular_z,
              active_threshold,
              zone.simulation_time_step,
              clearance);
          } else {
            if (!cfg.footprint_points.empty()) {
              clearance = point_to_polygon_distance(point, cfg.footprint_points);
            }
            collision_time = clearance / current_speed;
          }

          min_clearance = std::min(min_clearance, clearance);
          if (collision_time < min_collision_time) {
            min_collision_time = collision_time;
            best_point = point;
          }
        }
        if (cfg.ttc_visualization_enabled && std::isfinite(min_collision_time) &&
          (!ttc_visualization_.active || min_collision_time < ttc_visualization_.ttc_s))
        {
          ttc_visualization_.active = true;
          ttc_visualization_.zone_name = zone.name;
          ttc_visualization_.ttc_s = min_collision_time;
          ttc_visualization_.threshold_s = active_threshold;
          ttc_visualization_.min_clearance =
            std::isfinite(min_clearance) ? min_clearance : -1.0;
          ttc_visualization_.collision_point = best_point;
          sample_trajectory_visualization(
            cfg.footprint_points,
            chassis_state.prediction_linear_x,
            chassis_state.prediction_linear_y,
            chassis_state.prediction_angular_z,
            std::max(min_collision_time, active_threshold),
            zone.simulation_time_step,
            ttc_visualization_.trajectory_points,
            ttc_visualization_.footprint_samples);
        }
        if (std::isfinite(min_collision_time) && min_collision_time <= active_threshold) {
          abnormal = true;
          std::ostringstream oss;
          oss << "Collision approach alert: zone=" << zone.name
              << " ttc=" << min_collision_time
              << " threshold=" << active_threshold;
          if (std::isfinite(min_clearance)) {
            oss << " clearance=" << min_clearance;
          }
          if (use_trajectory_ttc) {
            oss << " mode=trajectory";
          }
          reason = oss.str();
        }
      }
    } else {
      if (!zone_matches_motion_direction(zone.motion_direction, runtime_direction)) {
        continue;
      }
      double inside_weight = 0.0;
      size_t inside_count = 0;
      for (const auto & point : points) {
        if (!is_point_inside_polygon(point, zone.points)) {
          continue;
        }
        inside_count++;
        inside_weight += std::max(0.0, point.weight);
      }
      abnormal = inside_weight >= zone.min_points;
      if (abnormal) {
        std::ostringstream oss;
        oss << "Collision zone hit: zone=" << zone.name << " weighted_points=" << inside_weight
            << " raw_points=" << inside_count << " min_points=" << zone.min_points;
        reason = oss.str();
      }
    }

    std::string active_reason;
    if (update_multi_value_state(key, abnormal, reason, now, zone.min_hold_time_s, active_reason)) {
      append_zone_faults(cfg, zone, active_reason, faults, now);
    }
  }

  return faults;
}

}  // namespace nav2_monitor
