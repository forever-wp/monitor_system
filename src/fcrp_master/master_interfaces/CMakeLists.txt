cmake_minimum_required(VERSION 3.8)
project(master_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 定义消息和服务文件
rosidl_generate_interfaces(${PROJECT_NAME}
  # 基础消息类型
  "msg/ErrorCode.msg"
  "msg/RobotState.msg"
  "msg/StandardResponse.msg"

  # 导航相关消息
  "msg/NavigationStatus.msg"
  "msg/PlanStatus.msg"

  # 状态机相关消息
  "msg/SystemState.msg"
  "msg/StateTransition.msg"
  "msg/TaskStatus.msg"

  # 基础服务
  "srv/Ping.srv"

  # 导航相关服务
  "srv/NavigateToPose.srv"
  "srv/CancelNavigation.srv"
  "srv/GetNavigationStatus.srv"
  "srv/FollowPath.srv"
  "srv/NavigateThroughPoses.srv"
  "srv/StartPlan.srv"
  "srv/GetPlanStatus.srv"
  "srv/ChangeLocalizer.srv"
  "srv/SwitchMap.srv"

  # 感知相关服务
  "srv/VisionElevatorStatus.srv"

  # 数据包录制消息
  "msg/BagInfo.msg"

  # 数据包录制服务
  "srv/StartRecord.srv"
  "srv/EndRecord.srv"
  "srv/CancelRecord.srv"
  "srv/GetBagList.srv"
  "srv/DeleteBag.srv"

  # 路径录制服务
  "srv/StartRecordPath.srv"
  "srv/StopRecordPath.srv"
  "srv/CancelRecordPath.srv"

  # 异常信息相关服务
  "srv/ReportException.srv"

  DEPENDENCIES std_msgs geometry_msgs nav_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  # 包含测试目录的CMakeLists.txt
  add_subdirectory(test)
  
  # 代码质量检查
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package() 
