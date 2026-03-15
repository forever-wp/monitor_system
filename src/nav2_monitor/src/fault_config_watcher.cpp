#include "nav2_monitor/fault_config_watcher.hpp"

#include <system_error>

namespace nav2_monitor
{

void FaultConfigWatcher::configure(const std::string & configured_path, const std::string & resolved_path)
{
  if (configured_path_ != configured_path || resolved_path_ != resolved_path) {
    initialized_ = false;
  }
  configured_path_ = configured_path;
  resolved_path_ = resolved_path;
}

void FaultConfigWatcher::sync_current_state()
{
  last_state_ = capture_current_state();
  initialized_ = true;
}

bool FaultConfigWatcher::poll_changed()
{
  const auto current = capture_current_state();
  if (!initialized_) {
    last_state_ = current;
    initialized_ = true;
    return true;
  }

  const bool changed =
    current.exists != last_state_.exists ||
    (current.exists && current.mtime != last_state_.mtime);
  if (changed) {
    last_state_ = current;
  }
  return changed;
}

bool FaultConfigWatcher::enabled() const
{
  return !configured_path_.empty();
}

const std::string & FaultConfigWatcher::configured_path() const
{
  return configured_path_;
}

const std::string & FaultConfigWatcher::resolved_path() const
{
  return resolved_path_;
}

FaultConfigWatcher::FileState FaultConfigWatcher::capture_current_state() const
{
  FileState state;
  if (resolved_path_.empty()) {
    return state;
  }

  std::error_code ec;
  state.exists = std::filesystem::exists(resolved_path_, ec);
  if (ec || !state.exists) {
    state.exists = false;
    return state;
  }

  state.mtime = std::filesystem::last_write_time(resolved_path_, ec);
  if (ec) {
    state.exists = false;
    state.mtime = std::filesystem::file_time_type{};
  }
  return state;
}

}  // namespace nav2_monitor
