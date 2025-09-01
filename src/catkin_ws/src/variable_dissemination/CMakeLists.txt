cmake_minimum_required(VERSION 3.8)
project(vardis)

# Add the executable
add_executable(binary
        main.cpp
        Helpers.cpp
        DatabaseManager.cpp
        VarDisQueue.cpp
        VarDisProtocol.cpp
)

# Link the necessary libraries
target_link_libraries(binary PRIVATE tins tbb)