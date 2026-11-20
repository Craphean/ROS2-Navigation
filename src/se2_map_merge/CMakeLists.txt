cmake_minimum_required(VERSION 3.16)
project(se2_map_merge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Core pure-C++17 library (zero ROS/OpenCV/Eigen dependencies for maximum portability and speed)
add_library(se2_map_merge_core STATIC
  src/phase_corr.cpp
  src/icp_2d.cpp
  src/cloud_filter.cpp
)
target_include_directories(se2_map_merge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(se2_map_merge_core PRIVATE -O3 -Wall -Wextra -Wpedantic)

# Standalone C++ unit test & speed benchmark binary (builds both with and without ROS)
include(CTest)
if(BUILD_TESTING)
  add_executable(test_se2_map_merge test/test_se2_map_merge.cpp)
  target_link_libraries(test_se2_map_merge PRIVATE se2_map_merge_core)
  target_compile_options(test_se2_map_merge PRIVATE -O3 -Wall -Wextra)
  add_test(NAME test_se2_map_merge COMMAND test_se2_map_merge)
endif()

# ROS 2 nodes built automatically when invoked via colcon / ament_cmake
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  find_package(rclcpp REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)

  add_executable(se2_map_merge_node src/se2_map_merge_node.cpp)
  target_link_libraries(se2_map_merge_node PRIVATE
    se2_map_merge_core
    rclcpp::rclcpp
    ${nav_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${tf2_msgs_TARGETS}
    tf2::tf2
    tf2_ros::tf2_ros
  )
  target_compile_options(se2_map_merge_node PRIVATE -O3 -Wall -Wextra)

  add_executable(cloud_merger_node src/cloud_merger_node.cpp)
  target_link_libraries(cloud_merger_node PRIVATE
    se2_map_merge_core
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    tf2::tf2
    tf2_ros::tf2_ros
  )
  target_compile_options(cloud_merger_node PRIVATE -O3 -Wall -Wextra)

  install(TARGETS
    se2_map_merge_core
    se2_map_merge_node
    cloud_merger_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY include/
    DESTINATION include
  )

  ament_export_include_directories(include)
  ament_package()
endif()
