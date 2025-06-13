cmake_minimum_required(VERSION 3.8)
project(lifecycle_controller)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(controller_msgs REQUIRED)
find_package(logging REQUIRED)

# Include directories
include_directories(
  include
)

# Source files
set(SOURCES
  src/cooperative.cpp
  src/dropout.cpp
  src/error.cpp
  src/fcu.cpp
  src/lifecycle_class.cpp
  src/main.cpp
  src/state.cpp
  src/config_reader.cpp
  src/input.cpp
  src/action_map.cpp
  src/status.cpp
)

# Add executable
add_executable(${PROJECT_NAME}_node ${SOURCES})

# Link libraries
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  tf2_ros
  px4_msgs
  std_srvs
  controller_msgs
  logging
)

# Include directories are handled by ament_target_dependencies via find_package
# No manual include paths necessary

# Check whether build is for the simulation or a real drone
# Check whether build is for the simulation or a real drone
if($ENV{SIM_BUILD})
  message(WARNING "Building for simulation environment")
  target_compile_definitions(lifecycle_controller_node PUBLIC SYSTEM_OFFSET=1)
endif()


# Install targets
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
