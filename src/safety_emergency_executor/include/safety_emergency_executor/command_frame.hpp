#ifndef SAFETY_EMERGENCY_EXECUTOR__COMMAND_FRAME_HPP_
#define SAFETY_EMERGENCY_EXECUTOR__COMMAND_FRAME_HPP_

namespace safety_emergency_executor
{

struct CommandFrame
{
  double speed{0.0};
  double angle{0.0};
  int acc{1000};
  int press{1000};
  int place{-1};
  int ulock{-1};
};

}  // namespace safety_emergency_executor

#endif  // SAFETY_EMERGENCY_EXECUTOR__COMMAND_FRAME_HPP_
