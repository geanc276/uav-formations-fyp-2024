#!/usr/bin/env python3

"""
Script Name: setup_systemd_services.py
Description: Automates the setup of systemd service files by copying them
             from the script's directory to /etc/systemd/system/,
             reloading systemd, and enabling & starting the services.
Author: OpenAI ChatGPT
Date: 2024-04-27
"""

import os
import sys
import shutil
import subprocess
from pathlib import Path
from typing import List

def echo_info(message: str):
    """Print informational messages in green."""
    print(f"\033[92m[INFO]\033[0m {message}")

def echo_warning(message: str):
    """Print warning messages in yellow."""
    print(f"\033[93m[WARNING]\033[0m {message}")

def echo_error(message: str):
    """Print error messages in red."""
    print(f"\033[91m[ERROR]\033[0m {message}")

def get_script_directory() -> Path:
    """
    Determines the directory where the script is located.

    Returns:
        Path: Absolute path to the script's directory.
    """
    return Path(__file__).resolve().parent

def find_service_files(script_dir: Path) -> List[Path]:
    """
    Finds all .service files in the given directory.

    Args:
        script_dir (Path): Directory to search for .service files.

    Returns:
        List[Path]: List of paths to .service files.
    """
    service_files = list(script_dir.glob("*.service"))
    return service_files

def backup_existing_service(service_path: Path):
    """
    Backs up an existing service file by appending a .bak extension.

    Args:
        service_path (Path): Path to the existing service file.
    """
    backup_path = service_path.with_suffix(service_path.suffix + '.bak')
    if backup_path.exists():
        echo_warning(f"Backup already exists for {service_path.name} at {backup_path}. Skipping backup.")
    else:
        echo_warning(f"Backing up existing service file: {service_path} to {backup_path}")
        shutil.copy2(service_path, backup_path)

def copy_service_file(src: Path, dest_dir: Path):
    """
    Copies a service file to the target systemd directory.

    Args:
        src (Path): Source path of the service file.
        dest_dir (Path): Destination directory for systemd service files.
    """
    dest = dest_dir / src.name
    if dest.exists():
        backup_existing_service(dest)
    echo_info(f"Copying {src.name} to {dest_dir}")
    shutil.copy2(src, dest)
    # Set permissions to 644
    os.chmod(dest, 0o644)
    echo_info(f"Set permissions for {dest} to 644")

def reload_systemd_daemon():
    """Reloads the systemd daemon to recognize new or updated service files."""
    echo_info("Reloading systemd daemon...")
    subprocess.run(["systemctl", "daemon-reload"], check=True)
    echo_info("systemd daemon reloaded successfully.")

def enable_service(service_name: str):
    """
    Enables a systemd service to start on boot.

    Args:
        service_name (str): Name of the service to enable.
    """
    echo_info(f"Enabling {service_name} to start on boot...")
    subprocess.run(["systemctl", "enable", service_name], check=True)
    echo_info(f"{service_name} enabled successfully.")

def start_service(service_name: str):
    """
    Starts a systemd service immediately.

    Args:
        service_name (str): Name of the service to start.
    """
    echo_info(f"Starting {service_name}...")
    subprocess.run(["systemctl", "start", service_name], check=True)
    echo_info(f"{service_name} started successfully.")

def check_service_status(service_name: str):
    """
    Checks and displays the status of a systemd service.

    Args:
        service_name (str): Name of the service to check.
    """
    echo_info(f"Checking the status of {service_name}...")
    try:
        subprocess.run(["systemctl", "status", service_name, "--no-pager"], check=True)
    except subprocess.CalledProcessError:
        echo_warning(f"{service_name} is not running as expected. Please check the logs for more details.")

# def make_files():

#     subprocess.run(["sudo","./setup.bash"], check=True)

def make_path(path : str):
    echo_info(f"Creating path: {path}")
    if not os.path.exists(path):
        echo_warning(f"Path {path} does not exist. Creating it now.")
        os.makedirs(path, exist_ok=True)
        echo_warning(f"Path {path} created successfully.")

def make_files():
    drone_id = subprocess.check_output(["hostname"]).decode().strip().split('-')[-1]
    config_folder_path = "/home/swarm_software/config"
    state_folder_path = "/home/swarm_software/state"

    config_content = f"""DRONE_ID={drone_id}\nDRONE_CONFIG_FOLDER_PATH={config_folder_path}\nDRONE_STATE_FOLDER_PATH={state_folder_path}\nROS_DOMAIN_ID=0\n
    """

    config_file_path = Path("/etc/drone_config.conf")
    config_file_path.write_text(config_content)

    subprocess.run(["sudo", "chown", "root:drone_swarm", str(config_file_path)], check=True)
    subprocess.run(["sudo", "chmod", "660", str(config_file_path)], check=True)

def setup_services(service_files: List[Path], target_dir: Path):
    """
    Sets up systemd services by copying, enabling, and starting them.

    Args:
        service_files (List[Path]): List of service file paths to set up.
        target_dir (Path): Destination directory for systemd service files.
    """


    make_files()

    for service_file in service_files:
        copy_service_file(service_file, target_dir)
    
    reload_systemd_daemon()
    
    for service_file in service_files:
        service_name = service_file.name
        # enable_service(service_name)
        # start_service(service_name)
        check_service_status(service_name)


def move_wrapper_file(name):
    wrapper_file_path = Path(f"./{name}")
    wrapper_dest_path = Path(f"/home/swarm_software/launch/{name}")
    if wrapper_dest_path.exists():
        backup_existing_service(wrapper_dest_path)
    echo_info(f"Copying {wrapper_file_path.name} to {wrapper_dest_path}")
    shutil.copy2(wrapper_file_path, wrapper_dest_path)
    # Set permissions to 755
    os.chmod(wrapper_dest_path, 0o755)
    echo_info(f"Set permissions for {wrapper_dest_path} to 755")


def setup_bag():
    log_path = Path("/home/swarm_software/logs/bag")
    make_path(log_path)
    init_bag_path = Path(f"{log_path}/init_bag.sh")
    if init_bag_path.exists():
        backup_existing_service(init_bag_path)
    echo_info(f"Copying init_bag.sh to {init_bag_path}")
    shutil.copy2(Path("./init_bag.sh"), init_bag_path)
    # Set permissions to 755
    os.chmod(init_bag_path, 0o755)
    echo_info(f"Set permissions for {init_bag_path} to 755")
    
    cleanup_bag_path = Path(f"{log_path}/cleanup_bag.sh")
    if cleanup_bag_path.exists():
        backup_existing_service(cleanup_bag_path)
    echo_info(f"Copying cleanup_bag.sh to {cleanup_bag_path}")
    shutil.copy2(Path("./cleanup_bag.sh"), cleanup_bag_path)
    # Set permissions to 755
    os.chmod(cleanup_bag_path, 0o755)
    echo_info(f"Set permissions for {cleanup_bag_path} to 755")



def main():
    try:
        # Step 1: Determine the script's directory
        script_dir = get_script_directory()
        echo_info(f"Script is located in: {script_dir}")

        # Step 2: Find all .service files in the script's directory
        service_files = find_service_files(script_dir)
        if not service_files:
            echo_error(f"No .service files found in {script_dir}. Please ensure all service files are present.")
            sys.exit(1)
        echo_info(f"Found {len(service_files)} service file(s): {[file.name for file in service_files]}")

        # Step 3: Define target directory for systemd service files
        target_dir = Path("/etc/systemd/system/")
        if not target_dir.exists():
            echo_error(f"Target systemd directory {target_dir} does not exist.")
            sys.exit(1)

        # Step 4: move wrapper file
        move_wrapper_file("ros2_launch_wrapper.sh")
        move_wrapper_file("target_launch_wrapper.sh")

        setup_bag()
        
        # Step 5: Setup services
        setup_services(service_files, target_dir)

        echo_info("All services have been set up successfully.")

    except shutil.SameFileError as e:
        echo_error(f"Source and destination represent the same file: {e}")
        sys.exit(1)
    except PermissionError as e:
        echo_error(f"Permission denied: {e}. Please run this script with sudo or as root.")
        sys.exit(1)
    except subprocess.CalledProcessError as e:
        echo_error(f"An error occurred while executing a subprocess command: {e}")
        sys.exit(1)
    except Exception as e:
        echo_error(f"An unexpected error occurred: {e}")
        sys.exit(1)

if __name__ == "__main__":
    if os.geteuid() != 0:
        echo_error("This script must be run as root. Please use sudo.")
        sys.exit(1)
    main()
