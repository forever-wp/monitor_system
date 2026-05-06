#ifndef NAV2_MONITOR__COLLISION_PREDICTION_ROUTER_HPP_
#define NAV2_MONITOR__COLLISION_PREDICTION_ROUTER_HPP_

#include <geometry_msgs/msg/twist.hpp>

#include <string>
#include <vector>

namespace nav2_monitor
{

struct CollisionPredictionRoutingConfig
{
  std::string prediction_speed_topic{"/cmd_vel"};
  std::string control_source_state_topic{"/control_source_state"};
  std::string prediction_speed_navigation_topic;
  std::string prediction_speed_miniapp_topic{"/cmd_vel_miniapp"};
  std::string prediction_speed_remote_topic{"/cmd_vel_remote"};
  std::string prediction_speed_other_topic{"/cmd_vel_other"};
};

class CollisionPredictionRouter
{
public:
  struct PredictionMotion
  {
    double linear_x{0.0};
    double linear_y{0.0};
    double angular_z{0.0};
  };

  struct SourceTopic
  {
    std::string source;
    std::string topic;
  };

  explicit CollisionPredictionRouter(
    CollisionPredictionRoutingConfig config = CollisionPredictionRoutingConfig{});

  const CollisionPredictionRoutingConfig & config() const;
  const std::string & active_source() const;
  const std::string & control_source_state_topic() const;
  std::string active_topic() const;
  std::string topic_for_source(const std::string & source) const;
  std::vector<SourceTopic> subscribed_sources() const;
  bool update_active_source(const std::string & raw_source);
  bool should_accept_source(const std::string & raw_source) const;
  bool is_known_source(const std::string & raw_source) const;

  static std::string normalize_source(const std::string & raw_source);
  static PredictionMotion extract_prediction_motion(
    const std::string & raw_source,
    const geometry_msgs::msg::Twist & msg);

private:
  static bool source_uses_embedded_command_fields(const std::string & normalized_source);
  static bool has_embedded_command_fields(const geometry_msgs::msg::Twist & msg);

  CollisionPredictionRoutingConfig config_;
  std::string active_source_{"navigation"};
};

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__COLLISION_PREDICTION_ROUTER_HPP_
