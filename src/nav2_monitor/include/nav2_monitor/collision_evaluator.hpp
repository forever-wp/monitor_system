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
    std::string last_reason;
  };

  static bool is_point_inside_polygon(
    const CollisionPoint & point,
    const std::vector<CollisionPoint> & polygon);
  bool update_multi_value_state(
    const std::string & key,
    bool abnormal,
    const std::string & reason,
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
