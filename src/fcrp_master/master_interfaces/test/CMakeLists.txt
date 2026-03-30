# robot_fc_ws/src/master_interfaces/test/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)

# 查找测试依赖
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

# 添加 GTest 单元测试
ament_add_gtest(test_interfaces test_interfaces.cpp)

if(TARGET test_interfaces)
  ament_target_dependencies(test_interfaces
    rclcpp
    geometry_msgs
    builtin_interfaces
  )
  
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(test_interfaces "${cpp_typesupport_target}")
endif()