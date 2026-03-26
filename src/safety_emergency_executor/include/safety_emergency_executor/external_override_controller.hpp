#ifndef SAFETY_EMERGENCY_EXECUTOR__EXTERNAL_OVERRIDE_CONTROLLER_HPP_
#define SAFETY_EMERGENCY_EXECUTOR__EXTERNAL_OVERRIDE_CONTROLLER_HPP_

#include <string>

namespace safety_emergency_executor
{

struct OverrideChange
{
  bool state_changed{false};
  bool publish_zero_command{false};
  std::string message;
};

class ExternalOverrideController
{
public:
  OverrideChange set_manual_override(bool active);
  bool manual_override_active() const;

private:
  bool manual_override_active_{false};
};

}  // namespace safety_emergency_executor

#endif  // SAFETY_EMERGENCY_EXECUTOR__EXTERNAL_OVERRIDE_CONTROLLER_HPP_
