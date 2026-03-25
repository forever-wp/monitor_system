#include "nav2_monitor/task_status_mapper.hpp"

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

void TaskStatusMapper::configure(const std::map<std::string, std::string> & code_to_task_mappings)
{
  code_to_task_mappings_.clear();
  for (const auto & entry : code_to_task_mappings) {
    const std::string code = trim_copy(entry.first);
    const std::string task_name = trim_copy(entry.second);
    if (!code.empty() && !task_name.empty()) {
      code_to_task_mappings_[code] = task_name;
    }
  }
}

std::string TaskStatusMapper::resolve_task_for_code(const std::string & code) const
{
  const std::string normalized = trim_copy(code);
  const auto it = code_to_task_mappings_.find(normalized);
  if (it == code_to_task_mappings_.end()) {
    return "";
  }
  return it->second;
}

bool TaskStatusMapper::has_mapping_for_code(const std::string & code) const
{
  return !resolve_task_for_code(code).empty();
}

const std::map<std::string, std::string> & TaskStatusMapper::mappings() const
{
  return code_to_task_mappings_;
}

}  // namespace nav2_monitor
