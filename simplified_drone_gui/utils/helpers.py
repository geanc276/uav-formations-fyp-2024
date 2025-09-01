# utils/helpers.py
import tkinter as tk
from tkinter import filedialog

#utils/helpers.py

import subprocess
import sys
import os




def parse_commands_supported(properties):
    """
    Extract and parse the 'commands_supported' field from properties.
    Converts bytes to string if necessary and returns a comma-separated string.
    """
    commands = properties.get('commands_supported', b'') if isinstance(properties.get('commands_supported'), bytes) else properties.get('commands_supported', '')
    if isinstance(commands, bytes):
        commands = commands.decode('utf-8')
    return commands

def file_selector(title="Select a file", filetypes=(("all files", "*.*"), ("text files", "*.txt"))):
    """
    Open a file dialog to select a file.
    """
    root = tk.Tk()
    root.withdraw()
    file_path = filedialog.askopenfilename(title=title, filetypes=filetypes)
    return file_path

def select_json_file():
    """
    Open a file dialog to select a JSON file.
    """
    return file_selector(title="Select a JSON file", filetypes=(("JSON files", "*.json"),))

def select_obj_file():
    """
    Open a file dialog to select an OBJ file.
    """
    return file_selector(title="Select an OBJ file", filetypes=(("OBJ files", "*.obj"),))

def select_image_file():
    """
    Open a file dialog to select an image file.
    """
    return file_selector(title="Select an image file", filetypes=(("Image files", "*.png *.jpg *.jpeg *.bmp"),))


# utils.py

import os
import subprocess
import sys
import re

# def run_colcon_build(progress_callback):
#     """
#     Runs the 'colcon build' command in the '../../swarm_ws' directory and updates progress via the callback.

#     :param progress_callback: Function to call with progress percentage and status message.
#     :return: True if build succeeded, False otherwise.
#     """
#     try:
#         # Determine the base directory
#         base_dir = os.path.abspath(os.path.dirname(__file__))

#         # Traverse up the directory tree to find the 'uav-formations' folder
#         while "uav-formations" not in os.path.basename(base_dir):
#             parent_dir = os.path.dirname(base_dir)
#             if parent_dir == base_dir:  # Stop if we've reached the root directory
#                 raise FileNotFoundError("Could not find 'uav-formations' directory.")
#             base_dir = parent_dir

#         # Navigate to the 'swarm_ws' directory
#         build_dir = os.path.join(base_dir, "swarm_ws")
#         if not os.path.exists(build_dir):
#             raise FileNotFoundError(f"'swarm_ws' directory not found at {build_dir}.")

#         # Change the working directory to the build directory
#         os.chdir(build_dir)

#         # Get the list of packages to be built
#         result = subprocess.run(
#             ["colcon", "list", "--names-only"],
#             stdout=subprocess.PIPE,
#             stderr=subprocess.PIPE,
#             text=True
#         )

#         if result.returncode != 0:
#             print(f"Error getting package list: {result.stderr}")
#             progress_callback(0, "Error getting package list.")
#             return False

#         package_list = result.stdout.strip().split('\n')
#         total_packages = len(package_list)

#         if total_packages == 0:
#             print("No packages to build.")
#             progress_callback(100, "No packages to build.")
#             return True

#         print(f"Total packages to build: {total_packages}")

#         # Start the colcon build process
#         process = subprocess.Popen(
#             ["colcon", "build","--symlink-install"],
#             stdout=subprocess.PIPE,
#             stderr=subprocess.STDOUT,
#             text=True,
#             bufsize=1,
#             universal_newlines=True
#         )

#         # Regular expressions to detect package start and finish
#         start_pattern = re.compile(r"^Starting >>> (.+)$")
#         finish_pattern = re.compile(r"^Finished <<< (.+) \[.*\]$")

#         built_packages = 0

#         for line in process.stdout:
#             print(line, end='')  # Optionally print output to console

#             # Check if a package build has started
#             start_match = start_pattern.match(line)
#             if start_match:
#                 package_name = start_match.group(1)
#                 status_message = f"Building package: {package_name}"
#                 progress = (built_packages / total_packages) * 100
#                 progress_callback(progress, status_message)

#             # Check if a package build has finished
#             finish_match = finish_pattern.match(line)
#             if finish_match:
#                 package_name = finish_match.group(1)
#                 built_packages += 1
#                 progress = (built_packages / total_packages) * 100
#                 status_message = f"Finished building {package_name}. Progress: {progress:.2f}%"
#                 progress_callback(progress, status_message)

#         process.wait()

#         # Ensure progress bar reaches 100% at the end
#         progress_callback(100, "Build process completed.")

#         return process.returncode == 0

#     except Exception as e:
#         print(f"Error during colcon build: {e}")
#         progress_callback(0, f"Error: {e}")
#         return False

# def run_zip_script(path = "install"):
#     """
#     Runs the 'zip' Python script.

#     :return: True if script succeeded, False otherwise.
#     """
#     try:
#         # Ensure the script is in the same directory as this script
#         # Determine the base directory
#         base_dir = os.path.abspath(os.path.dirname(__file__))

#         # Traverse up the directory tree to find the 'uav-formations' folder
#         while "uav-formations" not in os.path.basename(base_dir):
#             parent_dir = os.path.dirname(base_dir)
#             if parent_dir == base_dir:  # Stop if we've reached the root directory
#                 raise FileNotFoundError("Could not find 'uav-formations' directory.")
#             base_dir = parent_dir

#         # Navigate to the 'swarm_ws' directory
#         build_dir = os.path.join(base_dir, "swarm_ws")
#         script_path = os.path.join(build_dir, "create_drone_launch_zip.py")
#         print("-----------------------------------Script path:", script_path)
#         # Run the zip.py script
#         result = subprocess.run([sys.executable, script_path, "--path", path], capture_output=True, text=True)

#         if result.returncode == 0:
#             print("Zip script output:", result.stdout)
#             return True
#         else:
#             print("Zip script error:", result.stderr)
#             return False

#     except Exception as e:
#         print(f"Error running zip script: {e}")
#         return False

