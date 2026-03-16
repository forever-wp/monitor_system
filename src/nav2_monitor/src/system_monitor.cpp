#include "nav2_monitor/system_monitor.hpp"
#include <sys/statvfs.h>
#include <sstream>

namespace nav2_monitor
{

SystemMonitor::SystemMonitor() : last_total_(0), last_idle_(0), gpu_available_(false)
{
  if (nvmlInit_v2() == NVML_SUCCESS) {
    unsigned int count = 0;
    if (nvmlDeviceGetCount_v2(&count) == NVML_SUCCESS && count > 0) {
      gpu_available_ = true;
    }
  }
}

SystemMonitor::~SystemMonitor()
{
  if (gpu_available_) nvmlShutdown();
}

double SystemMonitor::get_cpu_usage()
{
  std::ifstream file("/proc/stat");
  std::string line;
  std::getline(file, line);

  std::istringstream ss(line);
  std::string cpu;
  uint64_t user, nice, system, idle, iowait, irq, softirq, steal;
  ss >> cpu >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;

  uint64_t total = user + nice + system + idle + iowait + irq + softirq + steal;
  uint64_t idle_time = idle + iowait;

  double usage = 0.0;
  if (last_total_ > 0) {
    uint64_t total_diff = total - last_total_;
    uint64_t idle_diff = idle_time - last_idle_;
    usage = total_diff > 0 ? 100.0 * (1.0 - (double)idle_diff / total_diff) : 0.0;
  }

  last_total_ = total;
  last_idle_ = idle_time;
  return usage;
}

double SystemMonitor::get_mem_usage()
{
  std::ifstream file("/proc/meminfo");
  std::string line;
  uint64_t total = 0, free = 0, buffers = 0, cached = 0;

  while (std::getline(file, line)) {
    std::istringstream ss(line);
    std::string key;
    uint64_t value;
    ss >> key >> value;
    if (key == "MemTotal:") total = value;
    else if (key == "MemFree:") free = value;
    else if (key == "Buffers:") buffers = value;
    else if (key == "Cached:") cached = value;
  }

  uint64_t used = total - free - buffers - cached;
  return total > 0 ? 100.0 * used / total : 0.0;
}

double SystemMonitor::get_disk_usage()
{
  struct statvfs stat;
  if (statvfs("/", &stat) != 0) return 0.0;
  uint64_t total = stat.f_blocks * stat.f_frsize;
  uint64_t free = stat.f_bfree * stat.f_frsize;
  return total > 0 ? 100.0 * (total - free) / total : 0.0;
}

double SystemMonitor::get_cpu_temp()
{
  std::ifstream file("/sys/class/thermal/thermal_zone0/temp");
  int temp;
  if (file >> temp) return temp / 1000.0;
  return 0.0;
}

double SystemMonitor::get_gpu_usage()
{
  if (!gpu_available_) return -1.0;
  nvmlDevice_t dev;
  if (nvmlDeviceGetHandleByIndex_v2(0, &dev) != NVML_SUCCESS) return -1.0;
  nvmlUtilization_t util;
  if (nvmlDeviceGetUtilizationRates(dev, &util) != NVML_SUCCESS) return -1.0;
  return util.gpu;
}

double SystemMonitor::get_gpu_temp()
{
  if (!gpu_available_) return -1.0;
  nvmlDevice_t dev;
  if (nvmlDeviceGetHandleByIndex_v2(0, &dev) != NVML_SUCCESS) return -1.0;
  unsigned int temp;
  if (nvmlDeviceGetTemperature(dev, NVML_TEMPERATURE_GPU, &temp) != NVML_SUCCESS) return -1.0;
  return temp;
}

double SystemMonitor::get_gpu_mem()
{
  if (!gpu_available_) return -1.0;
  nvmlDevice_t dev;
  if (nvmlDeviceGetHandleByIndex_v2(0, &dev) != NVML_SUCCESS) return -1.0;
  nvmlMemory_t mem;
  if (nvmlDeviceGetMemoryInfo(dev, &mem) != NVML_SUCCESS) return -1.0;
  return 100.0 * mem.used / mem.total;
}

}  // namespace nav2_monitor
