cmake_minimum_required(VERSION 3.8)
project(fastlio_go2w_fusion)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(fusion_core src/fusion_core.cpp src/pointcloud_parser.cpp)
target_include_directories(fusion_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(fusion_core sensor_msgs)

add_executable(dual_lidar_fusion_node src/dual_lidar_fusion_node.cpp)
target_link_libraries(dual_lidar_fusion_node fusion_core)
ament_target_dependencies(dual_lidar_fusion_node
  diagnostic_msgs
  livox_ros_driver2
  rclcpp
  sensor_msgs)

install(TARGETS fusion_core dual_lidar_fusion_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_fusion_core test/test_fusion_core.cpp)
  target_link_libraries(test_fusion_core fusion_core)

  ament_add_gtest(test_pointcloud_parser test/test_pointcloud_parser.cpp)
  target_link_libraries(test_pointcloud_parser fusion_core)
  ament_target_dependencies(test_pointcloud_parser sensor_msgs)
endif()

ament_export_include_directories(include)
ament_export_libraries(fusion_core)
ament_export_dependencies(sensor_msgs)
ament_package()
