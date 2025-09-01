cmake_minimum_required(VERSION 3.8)
project(target)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(shared_packages REQUIRED)
find_package(logging REQUIRED)
find_package(controller_msgs REQUIRED)


message("all environment variables: $ENV{TARGET}")
message("TARGET: $ENV{TARGET}")

# Decide which target to use.
if("$ENV{TARGET}" STREQUAL "virtual")  # Virtual Target
  message("Target Selected: Virtual")
  set(TARGET_SOURCES
    src/targets/virtual_target.cpp
    src/libs/pose.cpp
    src/libs/global_pose.cpp
    src/libs/state.cpp
  )
elseif("$ENV{TARGET}" STREQUAL "drone")  # Drone Target
  message("Target Selected: Drone")
  set(TARGET_SOURCES
    src/targets/drone_target.cpp
  )
elseif("$ENV{TARGET}" STREQUAL "gps")  # GPS Target
  message("Target Selected: GPS")
  set(TARGET_SOURCES
    src/targets/gps_target.cpp
  )
else()
  message(WARNING "No valid target selected, defaulting to drone target")
  set(TARGET_SOURCES
    src/targets/drone_target.cpp
  )
endif()




# Header files
set(HEADERS
  include/target/target_node.hpp
  include/target/libs/pose.hpp
  include/target/libs/global_pose.hpp
  include/target/libs/state.hpp
)

# Add node executable
add_executable(target_node ${TARGET_SOURCES})


# Check whether build is for the simulation or a real drone
# Check whether build is for the simulation or a real drone
if($ENV{SIM_BUILD})
  message(WARNING "Building for simulation environment")
  target_compile_definitions(target_node PUBLIC SIM_BUILD=1)
endif()




# Link libraries (add your necessary libraries)
ament_target_dependencies(target_node
  rclcpp
  px4_msgs
  std_msgs
  shared_packages
  controller_msgs
  logging
)

# Include directories
target_include_directories(target_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(target_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(DIRECTORY include/
  DESTINATION include/)

install(TARGETS target_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
