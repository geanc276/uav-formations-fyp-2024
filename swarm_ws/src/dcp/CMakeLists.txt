cmake_minimum_required(VERSION 3.8)
project(dcp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(dcp_msgs REQUIRED)
find_library(TINS_LIBRARY tins)
find_package(rcl_interfaces REQUIRED)

include_directories(
        include
        ${rclcpp_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
        ${rclcpp_components_INCLUDE_DIRS}
        ${dcp_msgs_INCLUDE_DIRS}
        ${rcl_interfaces_INCLUDE_DIRS}
)

add_library(BeaconingClient src/beaconing_client/beaconingClient.cpp)
add_library(SRPClient src/srp_client/srpClient.cpp)
add_library(VarDisClient src/variable_dissemination/VarDisClient.cpp)

add_library(VarDisQueue src/variable_dissemination/VarDisQueue.cpp)
add_library(DatabaseManager src/variable_dissemination/DatabaseManager.cpp)
add_library(Helpers src/variable_dissemination/Helpers.cpp)
add_library(neighbourTable src/neighbour_table/neighbourTable.cpp)

add_library(beaconing_client_node_component SHARED src/beaconing_client/beaconingClientNode.cpp)
add_library(srp_client_node_component SHARED src/srp_client/srpClientNode.cpp)
add_library(vardis_client_node_component SHARED src/variable_dissemination/VarDisClientNode.cpp)

target_compile_options(BeaconingClient PRIVATE -fPIC)
target_compile_options(SRPClient PRIVATE -fPIC)
target_compile_options(VarDisClient PRIVATE -fPIC)
target_compile_options(VarDisQueue PRIVATE -fPIC)
target_compile_options(DatabaseManager PRIVATE -fPIC)
target_compile_options(Helpers PRIVATE -fPIC)
target_compile_options(neighbourTable PRIVATE -fPIC)

# Add the executable
add_executable(binary
        src/variable_dissemination/main.cpp
        src/variable_dissemination/Helpers.cpp
        src/variable_dissemination/DatabaseManager.cpp
        src/variable_dissemination/VarDisQueue.cpp
        src/variable_dissemination/VarDisClient.cpp
)

# Link the necessary libraries
target_link_libraries(binary PRIVATE tins tbb ${rclcpp_LIBRARIES})

target_link_libraries(
        beaconing_client_node_component
        BeaconingClient
        neighbourTable
        ${TINS_LIBRARY}
        ${rclcpp_LIBRARIES}
)

target_link_libraries(
        srp_client_node_component
        SRPClient
        neighbourTable
        Helpers
        ${TINS_LIBRARY}
        ${rclcpp_LIBRARIES}
)

target_link_libraries(
        vardis_client_node_component
        VarDisClient
        VarDisQueue
        DatabaseManager
        Helpers
        ${TINS_LIBRARY}
        ${rclcpp_LIBRARIES}
)

target_link_libraries(
        VarDisClient
        VarDisQueue
        DatabaseManager
        Helpers
        ${TINS_LIBRARY}
        ${rclcpp_LIBRARIES}
        tbb  # Ensure TBB is linked here
)

ament_target_dependencies(
        BeaconingClient
        rclcpp
        std_msgs
        dcp_msgs
)

ament_target_dependencies(
        SRPClient
        rclcpp
        std_msgs
        dcp_msgs
)

ament_target_dependencies(
        VarDisClient
        rclcpp
        std_msgs
        dcp_msgs
)

ament_target_dependencies(
        beaconing_client_node_component
        rclcpp
        rclcpp_components
        std_msgs
        dcp_msgs
)

ament_target_dependencies(
        srp_client_node_component
        rclcpp
        rclcpp_components
        std_msgs
        dcp_msgs
)

ament_target_dependencies(
        vardis_client_node_component
        rclcpp
        rclcpp_components
        std_msgs
        dcp_msgs
)

rclcpp_components_register_node(
        beaconing_client_node_component
        PLUGIN "dcp::BeaconingClientNode"
        EXECUTABLE beaconing_client_node
)

rclcpp_components_register_node(
        srp_client_node_component
        PLUGIN "dcp::SRPClientNode"
        EXECUTABLE srp_client_node
)

rclcpp_components_register_node(
        vardis_client_node_component
        PLUGIN "dcp::VarDisClientNode"
        EXECUTABLE vardis_client_node
)

ament_export_dependencies(roscpp std_msgs dcp_msgs)
ament_export_include_directories(include/dcp)
ament_export_libraries(dcp)
ament_export_targets(export_beaconing_client_node_component)
ament_export_targets(export_srp_client_node_component)
ament_export_targets(export_vardis_client_node_component)

# Install targets
install(TARGETS srp_client_node_component
        SRPClient
        neighbourTable
        Helpers
        EXPORT export_srp_client_node_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS beaconing_client_node_component
        BeaconingClient
        neighbourTable
        EXPORT export_beaconing_client_node_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS vardis_client_node_component
        VarDisClient
        VarDisQueue
        DatabaseManager
        Helpers
        EXPORT export_vardis_client_node_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS binary
        DESTINATION bin
)

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
