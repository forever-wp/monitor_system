#include "nav2_monitor/collision_prediction_router.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>

namespace nav2_monitor
{

namespace
{

constexpr double kEmbeddedFieldEpsilon = 1e-6;

std::string trim_copy(const std::string & value)
{
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1);
}

}  // namespace

CollisionPredictionRouter::CollisionPredictionRouter(CollisionPredictionRoutingConfig config)
: config_(std::move(config))
{
}

const CollisionPredictionRoutingConfig & CollisionPredictionRouter::config() const
{
  return config_;
}

const std::string & CollisionPredictionRouter::active_source() const
{
  return active_source_;
}

const std::string & CollisionPredictionRouter::control_source_state_topic() const
{
  return config_.control_source_state_topic;
}

std::string CollisionPredictionRouter::active_topic() const
{
  return topic_for_source(active_source_);
}

std::string CollisionPredictionRouter::topic_for_source(const std::string & source) const
{
  const auto normalized = normalize_source(source);
  if (normalized == "miniapp") {
    return config_.prediction_speed_miniapp_topic;
  }
  if (normalized == "remote") {
    return config_.prediction_speed_remote_topic;
  }
  if (normalized == "other") {
    return config_.prediction_speed_other_topic;
  }
  if (normalized == "navigation") {
    return config_.prediction_speed_navigation_topic.empty() ?
      config_.prediction_speed_topic :
      config_.prediction_speed_navigation_topic;
  }
  return "";
}

std::vector<CollisionPredictionRouter::SourceTopic>
CollisionPredictionRouter::subscribed_sources() const
{
  std::vector<SourceTopic> topics;
  topics.reserve(4);

  for (const auto & source : std::array<std::string, 4>{
         "navigation", "miniapp", "remote", "other"})
  {
    const auto topic = topic_for_source(source);
    if (topic.empty()) {
      continue;
    }
    topics.push_back(SourceTopic{source, topic});
  }

  return topics;
}

bool CollisionPredictionRouter::update_active_source(const std::string & raw_source)
{
  const auto normalized = normalize_source(raw_source);
  if (!is_known_source(normalized) || normalized == active_source_) {
    return false;
  }

  active_source_ = normalized;
  return true;
}

bool CollisionPredictionRouter::should_accept_source(const std::string & raw_source) const
{
  return normalize_source(raw_source) == active_source_;
}

bool CollisionPredictionRouter::is_known_source(const std::string & raw_source) const
{
  const auto normalized = normalize_source(raw_source);
  return normalized == "navigation" ||
         normalized == "miniapp" ||
         normalized == "remote" ||
         normalized == "other";
}

std::string CollisionPredictionRouter::normalize_source(const std::string & raw_source)
{
  auto normalized = trim_copy(raw_source);
  std::transform(
    normalized.begin(), normalized.end(), normalized.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return normalized;
}

CollisionPredictionRouter::PredictionMotion CollisionPredictionRouter::extract_prediction_motion(
  const std::string & raw_source,
  const geometry_msgs::msg::Twist & msg)
{
  const auto normalized = normalize_source(raw_source);
  PredictionMotion motion;
  motion.linear_x = msg.linear.x;
  motion.linear_y = msg.linear.y;
  motion.angular_z = msg.angular.z;

  if (source_uses_embedded_command_fields(normalized) && has_embedded_command_fields(msg)) {
    motion.linear_y = 0.0;
  }

  return motion;
}

bool CollisionPredictionRouter::source_uses_embedded_command_fields(
  const std::string & normalized_source)
{
  return normalized_source == "miniapp" ||
         normalized_source == "remote" ||
         normalized_source == "other";
}

bool CollisionPredictionRouter::has_embedded_command_fields(const geometry_msgs::msg::Twist & msg)
{
  return std::abs(msg.linear.y) > kEmbeddedFieldEpsilon ||
         std::abs(msg.linear.z) > kEmbeddedFieldEpsilon ||
         std::abs(msg.angular.x) > kEmbeddedFieldEpsilon ||
         std::abs(msg.angular.y) > kEmbeddedFieldEpsilon;
}

}  // namespace nav2_monitor
