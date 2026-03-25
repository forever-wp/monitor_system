#ifndef NAV2_MONITOR__TASK_STATUS_MAPPER_HPP_
#define NAV2_MONITOR__TASK_STATUS_MAPPER_HPP_

#include <map>
#include <string>

namespace nav2_monitor
{

class TaskStatusMapper
{
public:
  void configure(const std::map<std::string, std::string> & code_to_task_mappings);
  std::string resolve_task_for_code(const std::string & code) const;
  bool has_mapping_for_code(const std::string & code) const;
  const std::map<std::string, std::string> & mappings() const;

private:
  std::map<std::string, std::string> code_to_task_mappings_;
};

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__TASK_STATUS_MAPPER_HPP_
