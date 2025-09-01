import paramiko
import os
import zipfile
import shutil
from datetime import datetime

script_path = os.path.dirname(os.path.abspath(__file__))


def create_folder_to_zip(path=None):
    # Define paths
    config_folder = os.path.join(script_path, 'configs', 'solo_config')  # Updated to point to solo_config
    states_folder = os.path.join(script_path, 'configs_states')
    install_folder = os.path.join(script_path, path if path else 'install')
    
    
    launch_file_python = os.path.join(script_path, 'launch', 'drone.launch.py')
    launch_file_bash = os.path.join(script_path, 'launch', 'drone_launch.bash')

    launch_target_drone_python = os.path.join(script_path, 'launch', 'target.launch.py')
    launch_target_drone_bash = os.path.join(script_path, 'launch', 'drone_target_launch.bash')

    # Define the target folder to create
    drone_launch_folder = os.path.join(script_path, 'drone_launch')

    # Remove the existing folder if it exists
    if os.path.exists(drone_launch_folder):
        shutil.rmtree(drone_launch_folder)
        print(f"Removed existing directory: {drone_launch_folder}")

    # Create the new folder
    os.makedirs(drone_launch_folder)
    print(f"Created directory: {drone_launch_folder}")

    # Copy the solo_config folder as configs
    destination_configs_folder = os.path.join(drone_launch_folder, 'configs')
    shutil.copytree(config_folder, destination_configs_folder)
    print(f"Copied and renamed {config_folder} to {destination_configs_folder}")

    # Copy the other necessary folders and files
    shutil.copytree(install_folder, os.path.join(drone_launch_folder, 'install'))
    shutil.copytree(states_folder, os.path.join(drone_launch_folder, 'configs_states'))
    shutil.copy(launch_file_python, drone_launch_folder)
    shutil.copy(launch_file_bash, drone_launch_folder)
    shutil.copy(launch_target_drone_python, drone_launch_folder)
    shutil.copy(launch_target_drone_bash, drone_launch_folder)

    print(f"Copied {install_folder}, {states_folder}, {launch_file_python}, and {launch_file_bash} to {drone_launch_folder}")

    # Get the current date and time
    current_datetime = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    # Create a text file with the current date and time
    date_file_path = os.path.join(drone_launch_folder, 'software_date.txt')
    with open(date_file_path, 'w') as date_file:
        date_file.write(f"software date: {current_datetime}\n")
    print(f"Created date file: {date_file_path} with date: {current_datetime}")


def zip_folder(local_folder):
    """Zip the given local folder."""
    zip_filename = local_folder.rstrip('/').split('/')[-1] + '.zip'
    zip_path = os.path.join(script_path, zip_filename)

    with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
        for root, _, files in os.walk(local_folder):
            for file in files:
                file_path = os.path.join(root, file)
                zipf.write(file_path, os.path.relpath(file_path, local_folder))

    print(f"Folder zipped successfully: {zip_path}")
    return zip_path


import argparse


def get_args():
    parser = argparse.ArgumentParser(description='Create a zip file for the drone launch.')
    parser.add_argument('--path', type=str, default=None, help='The path to the install folder.')
    return parser.parse_args()


def main():
    args = get_args()
    path = args.path
    print(f"Creating drone launch zip with install folder path: {path}")
    create_folder_to_zip(path)
    zip_folder(os.path.join(script_path, 'drone_launch'))


if __name__ == '__main__':
    main()
