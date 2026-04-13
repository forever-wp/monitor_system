cmake_minimum_required(VERSION 3.8)
project(collision_voxel_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VoxelCell.msg"
  "msg/VoxelGrid.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

if(BUILD_TESTING)
  ament_add_gtest(test_sparse_voxel_grid
    test/test_sparse_voxel_grid.cpp
  )
  if(TARGET test_sparse_voxel_grid)
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(test_sparse_voxel_grid "${cpp_typesupport_target}")
  endif()
endif()

install(DIRECTORY include/ DESTINATION include)

ament_package()
