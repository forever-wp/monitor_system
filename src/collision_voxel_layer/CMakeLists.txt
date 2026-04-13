cmake_minimum_required(VERSION 3.8)
project(collision_voxel_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
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

include_directories(include)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(sparse_voxel_grid_core
  src/sparse_voxel_grid.cpp
)
ament_target_dependencies(sparse_voxel_grid_core rclcpp std_msgs)
target_link_libraries(sparse_voxel_grid_core "${cpp_typesupport_target}")

add_library(source_adapter_core
  src/source_adapter.cpp
)
ament_target_dependencies(source_adapter_core sensor_msgs tf2 tf2_ros tf2_sensor_msgs)

add_library(collision_voxel_layer_node_core
  src/collision_voxel_layer_node.cpp
)
ament_target_dependencies(
  collision_voxel_layer_node_core
  builtin_interfaces
  geometry_msgs
  message_filters
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  visualization_msgs)
target_link_libraries(
  collision_voxel_layer_node_core
  source_adapter_core
  sparse_voxel_grid_core)

add_executable(collision_voxel_layer_node
  src/main.cpp
)
ament_target_dependencies(collision_voxel_layer_node rclcpp)
target_link_libraries(collision_voxel_layer_node collision_voxel_layer_node_core)

if(BUILD_TESTING)
  ament_add_gtest(test_sparse_voxel_grid
    test/test_sparse_voxel_grid.cpp
  )
  if(TARGET test_sparse_voxel_grid)
    ament_target_dependencies(test_sparse_voxel_grid rclcpp std_msgs)
    target_link_libraries(test_sparse_voxel_grid sparse_voxel_grid_core "${cpp_typesupport_target}")
  endif()

  ament_add_gtest(test_source_adapter
    test/test_source_adapter.cpp
  )
  if(TARGET test_source_adapter)
    ament_target_dependencies(test_source_adapter sensor_msgs tf2_ros tf2_sensor_msgs)
    target_link_libraries(test_source_adapter source_adapter_core)
  endif()

  ament_add_gtest(test_collision_voxel_layer_node
    test/test_collision_voxel_layer_node.cpp
  )
  if(TARGET test_collision_voxel_layer_node)
    ament_target_dependencies(test_collision_voxel_layer_node rclcpp)
    target_link_libraries(test_collision_voxel_layer_node collision_voxel_layer_node_core)
  endif()
endif()

install(
  TARGETS
  sparse_voxel_grid_core
  source_adapter_core
  collision_voxel_layer_node_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib)
install(
  TARGETS
  collision_voxel_layer_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
