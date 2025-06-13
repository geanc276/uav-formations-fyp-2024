

# Get the directory where the script is located
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Change to the src directory inside the script location
pushd "$script_dir/src"

# Prompt the user for the package name
echo "Please enter the name of the package you want to create:"
read package_name

# Check if the user input was empty
if [[ -z "$package_name" ]]; then
  echo "Error: No package name provided. Exiting."
  popd # Return to the original directory before exiting
  exit 1
fi

# Run the ROS 2 package creation command
ros2 pkg create --build-type ament_cmake --license Apache-2.0 "$package_name"

# Return to the original directory
popd
