cmake_minimum_required(VERSION 3.20)
project(controller_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PidSet.msg"
  "msg/RefPointGlobalPosition.msg"
  "msg/ErrorThresholdSet.msg"
  "msg/StateSet.msg"
  "msg/RegisterDrone.msg"
  "msg/RegisterDroneAck.msg"
  "msg/UpdateDroneList.msg"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()