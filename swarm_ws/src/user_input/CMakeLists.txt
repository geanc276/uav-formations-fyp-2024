cmake_minimum_required(VERSION 3.20)
project(user_input)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_msgs REQUIRED)


# Find Readline
find_path(READLINE_INCLUDE_DIR readline/readline.h)
find_library(READLINE_LIBRARY NAMES readline)

if (NOT READLINE_INCLUDE_DIR OR NOT READLINE_LIBRARY)
  message(FATAL_ERROR "Readline library not found.")
endif()

# include directories
include_directories(include ${READLINE_INCLUDE_DIR})

# add the executable
add_executable(${PROJECT_NAME}_node src/user_input.cpp)

# link against ROS 2 packages and Readline
ament_target_dependencies(${PROJECT_NAME}_node 
  rclcpp
  std_msgs
  controller_msgs
)

target_link_libraries(${PROJECT_NAME}_node ${READLINE_LIBRARY})

# install the executable
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()