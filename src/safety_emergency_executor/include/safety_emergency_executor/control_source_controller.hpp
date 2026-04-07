#ifndef SAFETY_EMERGENCY_EXECUTOR__CONTROL_SOURCE_CONTROLLER_HPP_
#define SAFETY_EMERGENCY_EXECUTOR__CONTROL_SOURCE_CONTROLLER_HPP_

#include <string>

namespace safety_emergency_executor
{

struct ControlSourceChange
{
  bool success{false};
  bool changed{false};
  std::string active_source;
  std::string message;
};

class ControlSourceController
{
public:
  explicit ControlSourceController(std::string initial_source, bool auto_preempt_enabled);

  const std::string & active_source() const;
  bool accepts(const std::string & source) const;
  ControlSourceChange set_active_source(const std::string & source);
  static bool is_valid_source(const std::string & source);

private:
  std::string active_source_;
  bool auto_preempt_enabled_{false};
};

}  // namespace safety_emergency_executor

#endif  // SAFETY_EMERGENCY_EXECUTOR__CONTROL_SOURCE_CONTROLLER_HPP_
