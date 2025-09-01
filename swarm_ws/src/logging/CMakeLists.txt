cmake_minimum_required(VERSION 3.8)

project(logging)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Declare a static library named 'logging'
add_library(logging STATIC src/logging.cpp)

# Specify include directories for the library
target_include_directories(logging PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install the library and export set named 'logging'
install(TARGETS logging
  EXPORT logging
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Export the 'logging' target for other packages to find
ament_export_targets(logging HAS_LIBRARY_TARGET logging)
ament_export_dependencies(ament_cmake)

# Finalize the package
ament_package()
