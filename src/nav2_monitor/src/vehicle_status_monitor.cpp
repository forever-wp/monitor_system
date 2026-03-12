#include "nav2_monitor/vehicle_status_monitor.hpp"
#include <fstream>
#include <sstream>

namespace nav2_monitor
{
namespace
{
std::string trim(const std::string & value)
{
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1);
}
}  // namespace

VehicleStatusMonitor::VehicleStatusMonitor(const std::string& status_file)
  : status_file_(status_file)
{
}

VehicleStatus VehicleStatusMonitor::get_status()
{
  VehicleStatus status;
  status.valid = false;

  std::ifstream file(status_file_);
  if (!file.is_open()) {
    return status;
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  return parse_json(buffer.str());
}

VehicleStatus VehicleStatusMonitor::parse_json(const std::string& content)
{
  VehicleStatus status;
  status.valid = false;

  auto get_value = [&](const std::string& key) -> std::string {
    size_t pos = content.find("\"" + key + "\":");
    if (pos == std::string::npos) return "";
    pos = content.find(":", pos) + 1;
    size_t end = content.find_first_of(",}", pos);
    std::string val = trim(content.substr(pos, end - pos));

    if (val.empty()) return "";
    if (val.front() == '"') {
      val = val.substr(1, val.find_last_of('"') - 1);
    }
    return val;
  };

  try {
    status.current_waypoint_index = std::stoi(get_value("current_waypoint_index"));
    status.error_message = get_value("error_message");
    status.navigation_active = get_value("navigation_active") == "true";
    status.navigation_succeeded = get_value("navigation_succeeded") == "true";
    status.progress_percentage = std::stod(get_value("progress_percentage"));
    status.simple_status = get_value("simple_status");
    status.status = get_value("status");
    status.timestamp = std::stoll(get_value("timestamp"));
    status.total_waypoints = std::stoi(get_value("total_waypoints"));
    status.valid = true;
  } catch (...) {
    status.valid = false;
  }

  return status;
}

}  // namespace nav2_monitor
