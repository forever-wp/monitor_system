cmake_minimum_required(VERSION 3.8)
project(nav2_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MonitorStatus.msg"
  "msg/SupervisorCmd.msg"
  "msg/SafetyCmd.msg"
  "msg/FaultEvent.msg"
  "msg/AlgorithmFeedback.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

include_directories(include)

add_executable(nav2_monitor_node
  src/nav2_monitor_node.cpp
  src/system_monitor.cpp
  src/fault_detector.cpp
  src/fault_config_watcher.cpp
  src/task_fault_config_selector.cpp
  src/task_status_mapper.cpp
  src/fault_state_coordinator.cpp
  src/monitor_data_store.cpp
  src/monitor_reporter.cpp
  src/chassis_evaluator.cpp
  src/collision_evaluator.cpp
  src/feedback_rule_evaluator.cpp
  src/watch_topic_evaluator.cpp
  src/vehicle_status_monitor.cpp
)
ament_target_dependencies(nav2_monitor_node rclcpp std_msgs nav_msgs sensor_msgs geometry_msgs tf2_ros ament_index_cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(nav2_monitor_node "${cpp_typesupport_target}")
target_link_libraries(nav2_monitor_node nvidia-ml yaml-cpp)

install(TARGETS nav2_monitor_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fault_detector
    test/test_fault_detector.cpp
    src/fault_detector.cpp
    src/fault_config_watcher.cpp
    src/task_fault_config_selector.cpp
    src/task_status_mapper.cpp
    src/fault_state_coordinator.cpp
    src/monitor_data_store.cpp
    src/monitor_reporter.cpp
    src/chassis_evaluator.cpp
    src/collision_evaluator.cpp
    src/feedback_rule_evaluator.cpp
    src/watch_topic_evaluator.cpp
  )
  if(TARGET test_fault_detector)
    target_include_directories(test_fault_detector PRIVATE include)
    ament_target_dependencies(test_fault_detector rclcpp geometry_msgs)
    target_link_libraries(test_fault_detector yaml-cpp "${cpp_typesupport_target}")
  endif()
endif()

ament_package()
