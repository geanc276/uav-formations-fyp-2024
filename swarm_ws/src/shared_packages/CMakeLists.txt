cmake_minimum_required(VERSION 3.15)
project(shared_packages)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# find_package(logging REQUIRED)

# Create an INTERFACE library target for a header-only package.
add_library(${PROJECT_NAME} INTERFACE)

# Set the include directories.
target_include_directories(${PROJECT_NAME} INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

# # Propagate the logging dependency to consumers.
# ament_target_dependencies(${PROJECT_NAME} INTERFACE logging)

# # Alternatively, if logging provides a target, you can also link it:
# # target_link_libraries(${PROJECT_NAME} INTERFACE logging::logging)

# Install the header files.
install(DIRECTORY include/
	DESTINATION include
)

# Export the include directories so that dependent packages know where to find the headers.
ament_export_include_directories(include)

# Export the target so that downstream packages can use ament_target_dependencies(shared_packages).
install(
	TARGETS ${PROJECT_NAME}
	EXPORT ${PROJECT_NAME}Targets
	INCLUDES DESTINATION include
)
ament_export_targets(${PROJECT_NAME}Targets)

ament_package()