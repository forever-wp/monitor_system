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

bool CollisionEvaluator::update_multi_value_state(
  const std::string & key,
  bool abnormal,
  const std::string & reason,
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
    }
  } else {
    state.normal_count++;
    state.abnormal_count = 0;
    if (state.latched && state.normal_count >= multi_value_cfg_.recover_count) {
      state.latched = false;
      state.last_reason.clear();
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
  if (!cfg.enabled) {
    return faults;
  }

  const auto points = store.get_collision_points(now, cfg.source_timeout_s);
  if (points.empty()) {
    return faults;
  }

  const auto & chassis_state = store.get_chassis_state();
  const double current_speed = std::fabs(chassis_state.command_speed);

  for (const auto & zone : cfg.zones) {
    if (!zone.enabled || zone.points.size() < 3) {
      continue;
    }

    bool abnormal = false;
    std::string reason;
    if (zone.model == CollisionModelType::APPROACH) {
      if (current_speed > 1e-3) {
        double min_collision_time = std::numeric_limits<double>::infinity();
        for (const auto & point : points) {
          if (!is_point_inside_polygon(point, zone.points)) {
            continue;
          }
          const double distance = point_norm(point);
          min_collision_time = std::min(min_collision_time, distance / current_speed);
        }
        if (std::isfinite(min_collision_time) && min_collision_time <= zone.time_before_collision) {
          abnormal = true;
          std::ostringstream oss;
          oss << "Collision approach alert: zone=" << zone.name
              << " ttc=" << min_collision_time
              << " threshold=" << zone.time_before_collision;
          reason = oss.str();
        }
      }
    } else {
      size_t inside_count = 0;
      for (const auto & point : points) {
        if (is_point_inside_polygon(point, zone.points)) {
          inside_count++;
        }
      }
      abnormal = inside_count >= zone.min_points;
      if (abnormal) {
        std::ostringstream oss;
        oss << "Collision zone hit: zone=" << zone.name << " points=" << inside_count
            << " min_points=" << zone.min_points;
        reason = oss.str();
      }
    }

    std::string active_reason;
    const std::string key = cfg.module_name + "|collision:" + zone.name;
    if (update_multi_value_state(key, abnormal, reason, active_reason)) {
      append_zone_faults(cfg, zone, active_reason, faults, now);
    }
  }

  return faults;
}

}  // namespace nav2_monitor
