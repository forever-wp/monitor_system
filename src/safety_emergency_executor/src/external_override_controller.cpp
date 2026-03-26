#include "safety_emergency_executor/external_override_controller.hpp"

namespace safety_emergency_executor
{

OverrideChange ExternalOverrideController::set_manual_override(bool active)
{
  OverrideChange change;
  if (active == manual_override_active_) {
    change.message = active ? "manual override already active" : "manual override already inactive";
    return change;
  }

  manual_override_active_ = active;
  change.state_changed = true;
  change.publish_zero_command = active;
  change.message = active ? "manual override enabled" : "manual override disabled";
  return change;
}

bool ExternalOverrideController::manual_override_active() const
{
  return manual_override_active_;
}

}  // namespace safety_emergency_executor
