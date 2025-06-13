#!/bin/bash

# Check if ROS 2 environment is set up
if [ -z "$ROS_VERSION" ]; then
  echo "Error: ROS 2 environment not set up. Please source your ROS 2 setup.bash file."
  exit 1
fi

# Check if package name is provided as argument
if [ $# -eq 0 ]; then
  echo "Error: Package name not provided."
  echo "Usage: $0 <package_name>"
  exit 1
fi

# Navigate to the ROS 2 workspace directory
cd ~/swarm_ws # Change this to your ROS 2 workspace directory

# Build the package

colcon build --packages-select $1 --event-handlers console_direct+

# Check if build was successful
if [ $? -eq 0 ]; then
  echo "Build successful."
else
  echo "Error: Build failed."
  exit 1
fi

# Run tests for the package
# colcon test --packages-select $1 --rerun-failed --output-on-failure

# colcon test --packages-select $1
colcon test --packages-select $1 --test-result-base /media/psf/Home/Desktop/UC_Uni/year_4/FYP/git/uav-formations-fyp-2024/build/controller/test_results --retest-until-pass 3

# Check if tests were successful
if [ $? -eq 0 ]; then
  echo "Tests passed."
else
  echo "Error: Tests failed."
  exit 1
fi

exit 0
