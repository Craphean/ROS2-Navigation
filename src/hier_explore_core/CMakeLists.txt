cmake_minimum_required(VERSION 3.16)
project(hier_explore_core)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install ROS 2 executable entrypoint
install(
  PROGRAMS
    scripts/coordinator_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_hier_explore_core test)
endif()

ament_package()
