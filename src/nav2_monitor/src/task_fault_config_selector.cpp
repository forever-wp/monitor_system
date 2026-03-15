#include "nav2_monitor/task_fault_config_selector.hpp"

namespace nav2_monitor
{

void TaskFaultConfigSelector::configure(
  const std::string & default_fault_config,
  const std::map<std::string, std::string> & task_fault_configs)
{
  default_fault_config_ = default_fault_config;
  task_fault_configs_ = task_fault_configs;
}

bool TaskFaultConfigSelector::update_current_task(const std::string & task_name)
{
  const std::string normalized = task_name.empty() ? "default" : task_name;
  task_changed_ = (normalized != current_task_);
  current_task_ = normalized;
  return task_changed_;
}

std::string TaskFaultConfigSelector::resolve_fault_config_for_task() const
{
  const auto current_it = task_fault_configs_.find(current_task_);
  if (current_it != task_fault_configs_.end() && !current_it->second.empty()) {
    return current_it->second;
  }

  const auto default_it = task_fault_configs_.find("default");
  if (default_it != task_fault_configs_.end() && !default_it->second.empty()) {
    return default_it->second;
  }

  return default_fault_config_;
}

bool TaskFaultConfigSelector::has_task_changed() const
{
  return task_changed_;
}

void TaskFaultConfigSelector::clear_task_changed()
{
  task_changed_ = false;
}

const std::string & TaskFaultConfigSelector::current_task() const
{
  return current_task_;
}

}  // namespace nav2_monitor
