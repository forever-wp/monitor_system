#ifndef NAV2_MONITOR__TASK_STATUS_MESSAGE_ADAPTER_HPP_
#define NAV2_MONITOR__TASK_STATUS_MESSAGE_ADAPTER_HPP_

#include <string>

#include "master_interfaces/msg/task_status.hpp"

namespace nav2_monitor
{

class TaskStatusMessageAdapter
{
public:
  static std::string extract_code(const master_interfaces::msg::TaskStatus & msg);
};

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__TASK_STATUS_MESSAGE_ADAPTER_HPP_
