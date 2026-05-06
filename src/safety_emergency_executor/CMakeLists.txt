cmake_minimum_required(VERSION 3.8)
project(safety_emergency_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_monitor REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(jsoncpp REQUIRED jsoncpp)

include_directories(
  include
  ${jsoncpp_INCLUDE_DIRS}
)

add_executable(safety_emergency_executor_node
  src/main.cpp
  src/safety_emergency_executor_node.cpp
  src/control_source_controller.cpp
  src/velocity_converter.cpp
  src/pressure_adjuster.cpp
  src/safety_policy_executor.cpp
  src/linear_pressure_adjuster.cpp
)
ament_target_dependencies(safety_emergency_executor_node
  rclcpp
  rcl_interfaces
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs
  nav2_monitor
)
target_link_libraries(safety_emergency_executor_node
  ${jsoncpp_LIBRARIES}
)

install(TARGETS safety_emergency_executor_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control_source_controller
    test/test_control_source_controller.cpp
  )
  if(TARGET test_control_source_controller)
    target_include_directories(test_control_source_controller PRIVATE include)
    target_sources(test_control_source_controller PRIVATE
      src/control_source_controller.cpp
    )
  endif()
  ament_add_gtest(test_control_source_routing
    test/test_control_source_routing.cpp
    src/safety_emergency_executor_node.cpp
    src/control_source_controller.cpp
    src/velocity_converter.cpp
    src/pressure_adjuster.cpp
    src/safety_policy_executor.cpp
    src/linear_pressure_adjuster.cpp
  )
  if(TARGET test_control_source_routing)
    target_include_directories(test_control_source_routing PRIVATE include)
    ament_target_dependencies(test_control_source_routing
      rclcpp
      rcl_interfaces
      geometry_msgs
      nav_msgs
      sensor_msgs
      std_msgs
      std_srvs
      nav2_monitor
    )
    target_link_libraries(test_control_source_routing
      ${jsoncpp_LIBRARIES}
    )
    set_tests_properties(test_control_source_routing PROPERTIES
      ENVIRONMENT "HOME=/tmp;ROS_HOME=/tmp/.ros;ROS_LOG_DIR=/tmp/ros_logs")
  endif()
  ament_add_gtest(test_pipeline_components
    test/test_pipeline_components.cpp
    src/velocity_converter.cpp
    src/pressure_adjuster.cpp
    src/safety_policy_executor.cpp
    src/linear_pressure_adjuster.cpp
  )
  if(TARGET test_pipeline_components)
    target_include_directories(test_pipeline_components PRIVATE include)
    ament_target_dependencies(test_pipeline_components
      rclcpp
      geometry_msgs
      nav_msgs
      sensor_msgs
      std_msgs
      nav2_monitor
    )
    target_link_libraries(test_pipeline_components
      ${jsoncpp_LIBRARIES}
    )
    set_tests_properties(test_pipeline_components PROPERTIES
      ENVIRONMENT "HOME=/tmp;ROS_HOME=/tmp/.ros;ROS_LOG_DIR=/tmp/ros_logs")
  endif()
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
