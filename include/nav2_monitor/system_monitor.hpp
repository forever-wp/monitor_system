#ifndef NAV2_MONITOR__SYSTEM_MONITOR_HPP_
#define NAV2_MONITOR__SYSTEM_MONITOR_HPP_

#include <string>
#include <fstream>
#include <nvml.h>

namespace nav2_monitor
{

class SystemMonitor
{
public:
  SystemMonitor();
  ~SystemMonitor();

  double get_cpu_usage();
  double get_mem_usage();
  double get_disk_usage();
  double get_cpu_temp();
  double get_gpu_usage();
  double get_gpu_temp();
  double get_gpu_mem();

private:
  uint64_t last_total_, last_idle_;
  bool gpu_available_;
};

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__SYSTEM_MONITOR_HPP_
