#ifndef NAV2_MONITOR__VEHICLE_STATUS_MONITOR_HPP_
#define NAV2_MONITOR__VEHICLE_STATUS_MONITOR_HPP_

#include <string>
#include <fstream>

namespace nav2_monitor
{

struct VehicleStatus
{
  int current_waypoint_index;
  std::string error_message;
  bool navigation_active;
  bool navigation_succeeded;
  double progress_percentage;
  std::string simple_status;
  std::string status;
  int64_t timestamp;
  int total_waypoints;
  bool valid;
};

class VehicleStatusMonitor
{
public:
  VehicleStatusMonitor(const std::string& status_file);

  VehicleStatus get_status();

private:
  std::string status_file_;
  VehicleStatus parse_json(const std::string& content);
};

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__VEHICLE_STATUS_MONITOR_HPP_
