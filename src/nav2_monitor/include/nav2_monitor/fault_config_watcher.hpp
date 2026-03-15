#ifndef NAV2_MONITOR__FAULT_CONFIG_WATCHER_HPP_
#define NAV2_MONITOR__FAULT_CONFIG_WATCHER_HPP_

#include <filesystem>
#include <string>

namespace nav2_monitor
{

class FaultConfigWatcher
{
public:
  void configure(const std::string & configured_path, const std::string & resolved_path);
  void sync_current_state();
  bool poll_changed();

  bool enabled() const;
  const std::string & configured_path() const;
  const std::string & resolved_path() const;

private:
  struct FileState
  {
    bool exists{false};
    std::filesystem::file_time_type mtime{};
  };

  FileState capture_current_state() const;

  std::string configured_path_;
  std::string resolved_path_;
  bool initialized_{false};
  FileState last_state_{};
};

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__FAULT_CONFIG_WATCHER_HPP_
