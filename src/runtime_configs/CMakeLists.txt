cmake_minimum_required(VERSION 3.8)
project(runtime_configs)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config/
  DESTINATION /opt/ry/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_monitor_layout
    test/test_monitor_layout.py
  )
endif()

ament_package()
