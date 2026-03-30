#include "nav2_monitor/task_status_message_adapter.hpp"

#include <algorithm>
#include <cctype>

namespace nav2_monitor
{
namespace
{
std::string trim_copy(const std::string & value)
{
  const auto begin = std::find_if_not(value.begin(), value.end(), [](unsigned char ch) {
    return std::isspace(ch) != 0;
  });
  if (begin == value.end()) {
    return "";
  }

  const auto end = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char ch) {
    return std::isspace(ch) != 0;
  }).base();
  return std::string(begin, end);
}
}  // namespace

std::string TaskStatusMessageAdapter::extract_code(const master_interfaces::msg::TaskStatus & msg)
{
  return trim_copy(msg.status_code);
}

}  // namespace nav2_monitor
