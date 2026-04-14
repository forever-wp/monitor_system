#include "safety_emergency_executor/control_source_controller.hpp"

#include <utility>

namespace safety_emergency_executor
{

namespace
{
constexpr const char * kNavigation = "navigation";
constexpr const char * kMiniapp = "miniapp";
constexpr const char * kRemote = "remote";
constexpr const char * kOther = "other";
}  // namespace

ControlSourceController::ControlSourceController(
  std::string initial_source,
  bool auto_preempt_enabled)
: active_source_(is_valid_source(initial_source) ? std::move(initial_source) : std::string(
      kNavigation)),
  auto_preempt_enabled_(auto_preempt_enabled)
{
  (void)auto_preempt_enabled_;
}

const std::string & ControlSourceController::active_source() const
{
  return active_source_;
}

bool ControlSourceController::accepts(const std::string & source) const
{
  return source == active_source_;
}

ControlSourceChange ControlSourceController::set_active_source(const std::string & source)
{
  ControlSourceChange result;
  if (!is_valid_source(source)) {
    result.success = false;
    result.changed = false;
    result.active_source = active_source_;
    result.message = "invalid control source: " + source;
    return result;
  }

  result.success = true;
  result.changed = source != active_source_;
  active_source_ = source;
  result.active_source = active_source_;
  result.message = result.changed ?
    "control source switched to " + active_source_ :
    "control source already set to " + active_source_;
  return result;
}

bool ControlSourceController::is_valid_source(const std::string & source)
{
  return source == kNavigation || source == kMiniapp || source == kRemote || source == kOther;
}

}  // namespace safety_emergency_executor
