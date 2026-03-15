#ifndef NAV2_MONITOR__TASK_FAULT_CONFIG_SELECTOR_HPP_
#define NAV2_MONITOR__TASK_FAULT_CONFIG_SELECTOR_HPP_

#include <map>
#include <string>

namespace nav2_monitor
{

class TaskFaultConfigSelector
{
public:
  void configure(
    const std::string & default_fault_config,
    const std::map<std::string, std::string> & task_fault_configs);

  bool update_current_task(const std::string & task_name);
  std::string resolve_fault_config_for_task() const;
  bool has_task_changed() const;
  void clear_task_changed();

  const std::string & current_task() const;

private:
  std::string default_fault_config_;
  std::map<std::string, std::string> task_fault_configs_;
  std::string current_task_{"default"};
  bool task_changed_{false};
};

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__TASK_FAULT_CONFIG_SELECTOR_HPP_
