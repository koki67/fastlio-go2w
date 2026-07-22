cmake_minimum_required(VERSION 3.8)
project(fastlio_go2w_hesai)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(pointcloud_adapter_core src/pointcloud_adapter.cpp)
target_include_directories(pointcloud_adapter_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pointcloud_adapter_core sensor_msgs)

add_executable(hesai_pointcloud_adapter src/hesai_pointcloud_adapter_node.cpp)
target_link_libraries(hesai_pointcloud_adapter pointcloud_adapter_core)
ament_target_dependencies(hesai_pointcloud_adapter diagnostic_msgs rclcpp sensor_msgs)

install(TARGETS pointcloud_adapter_core hesai_pointcloud_adapter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pointcloud_adapter test/test_pointcloud_adapter.cpp)
  if(TARGET test_pointcloud_adapter)
    target_link_libraries(test_pointcloud_adapter pointcloud_adapter_core)
    ament_target_dependencies(test_pointcloud_adapter sensor_msgs)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(pointcloud_adapter_core)
ament_export_dependencies(sensor_msgs)
ament_package()
