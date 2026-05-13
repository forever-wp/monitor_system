#ifndef NAV2_MONITOR__CONFIG_PROFILE_SYNC_HPP_
#define NAV2_MONITOR__CONFIG_PROFILE_SYNC_HPP_

#include <sstream>
#include <string>

namespace nav2_monitor
{

struct ConfigProfileUpdate
{
  std::string task_name;
  std::string fault_config;
  std::string resolved_fault_config;
};

inline std::string config_profile_json_escape(const std::string & input)
{
  std::ostringstream oss;
  for (const auto ch : input) {
    switch (ch) {
      case '"': oss << "\\\""; break;
      case '\\': oss << "\\\\"; break;
      case '\n': oss << "\\n"; break;
      case '\r': oss << "\\r"; break;
      case '\t': oss << "\\t"; break;
      default: oss << ch; break;
    }
  }
  return oss.str();
}

inline std::string config_profile_update_to_json(const ConfigProfileUpdate & update)
{
  std::ostringstream oss;
  oss << '{'
      << "\"task_name\":\"" << config_profile_json_escape(update.task_name) << "\","
      << "\"fault_config\":\"" << config_profile_json_escape(update.fault_config) << "\","
      << "\"resolved_fault_config\":\"" << config_profile_json_escape(update.resolved_fault_config) << "\""
      << '}';
  return oss.str();
}

inline bool config_profile_extract_json_string(
  const std::string & json,
  const std::string & key,
  std::string & value)
{
  const std::string marker = "\"" + key + "\":";
  const auto marker_pos = json.find(marker);
  if (marker_pos == std::string::npos) {
    return false;
  }
  auto pos = json.find('"', marker_pos + marker.size());
  if (pos == std::string::npos) {
    return false;
  }
  ++pos;

  value.clear();
  bool escaped = false;
  for (; pos < json.size(); ++pos) {
    const char ch = json[pos];
    if (escaped) {
      switch (ch) {
        case 'n': value.push_back('\n'); break;
        case 'r': value.push_back('\r'); break;
        case 't': value.push_back('\t'); break;
        default: value.push_back(ch); break;
      }
      escaped = false;
      continue;
    }
    if (ch == '\\') {
      escaped = true;
      continue;
    }
    if (ch == '"') {
      return true;
    }
    value.push_back(ch);
  }
  return false;
}

inline bool parse_config_profile_update(
  const std::string & json,
  ConfigProfileUpdate & update)
{
  if (!config_profile_extract_json_string(json, "fault_config", update.fault_config)) {
    return false;
  }
  (void)config_profile_extract_json_string(json, "task_name", update.task_name);
  (void)config_profile_extract_json_string(
    json, "resolved_fault_config", update.resolved_fault_config);
  return !update.fault_config.empty();
}

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__CONFIG_PROFILE_SYNC_HPP_
