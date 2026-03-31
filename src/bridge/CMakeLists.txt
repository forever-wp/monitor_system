cmake_minimum_required(VERSION 3.8)
project(bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_monitor REQUIRED)

ament_python_install_package(bridge)

include_directories(include)

add_executable(bridge_cpp_node
  src/bridge_cpp_main.cpp
  src/sample_feedback_bridge.cpp
)
ament_target_dependencies(bridge_cpp_node
  rclcpp
  sensor_msgs
  std_msgs
  nav2_monitor
)

install(TARGETS bridge_cpp_node
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/bridge_py_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_sample_feedback_bridge
    test/test_sample_feedback_bridge.cpp
    src/sample_feedback_bridge.cpp
  )
  if(TARGET test_sample_feedback_bridge)
    target_include_directories(test_sample_feedback_bridge PRIVATE include)
    ament_target_dependencies(test_sample_feedback_bridge
      rclcpp
      sensor_msgs
      std_msgs
      nav2_monitor
    )
  endif()

  ament_add_pytest_test(test_config_driven_bridge
    test/test_config_driven_bridge.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )

  ament_add_pytest_test(test_monitor_ota_layout
    test/test_monitor_ota_layout.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
