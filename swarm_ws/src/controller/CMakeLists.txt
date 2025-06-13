cmake_minimum_required(VERSION 3.15)
project(controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nlohmann_json REQUIRED) # JSON dependency
find_package(std_srvs REQUIRED)
find_package(controller_msgs REQUIRED)
find_package(rcutils REQUIRED) # Logging dependency
find_package(logging REQUIRED)


# Source files
set(CONTROLLER_SOURCES
  src/controller_node.cpp
  src/config.cpp
  src/pose.cpp
  src/global_pose.cpp
  src/pid_manager.cpp
  src/state.cpp
  src/local_pose.cpp
  src/main.cpp
)

set(RP_SOURCES
  src/ref_point.cpp
  src/config.cpp
  src/global_pose.cpp
  src/state.cpp
  src/pose.cpp
  src/local_pose.cpp
)

# Header files
set(HEADERS
  include/controller/config.hpp
  include/controller/controller_node.hpp
  include/controller/pose.hpp
  include/controller/controller.hpp
  include/controller/global_pose.hpp
  include/controller/pid_manager.hpp
  include/controller/ref_point.hpp
  include/controller/state.hpp
  include/controller/local_pose.hpp
)

# Add node executable
add_executable(controller_node ${CONTROLLER_SOURCES})
add_executable(ref_point_node ${RP_SOURCES})

# Link libraries (add necessary libraries)
ament_target_dependencies(controller_node
  rclcpp
  std_msgs
  tf2_ros
  control_toolbox
  px4_msgs
  nlohmann_json # JSON dependency
  std_srvs
  controller_msgs
  rcutils # Logging dependency
  logging
)

ament_target_dependencies(ref_point_node
  rclcpp
  std_msgs
  tf2_ros
  control_toolbox
  px4_msgs
  nlohmann_json # JSON dependency
  controller_msgs
  rcutils # Logging dependency
  logging
)

# Include directories
target_include_directories(controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(ref_point_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Check whether build is for the simulation or a real drone
if($ENV{SIM_BUILD})
  message(WARNING "Building for simulation environment")
  target_compile_definitions(controller_node PUBLIC SYSTEM_OFFSET=1)
endif()

install(DIRECTORY include/
  DESTINATION include/)

# Install targets
install(TARGETS
  controller_node
  ref_point_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
