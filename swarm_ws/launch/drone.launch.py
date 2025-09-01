from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
import os



def get_first_file_with_extension(folder_path, extension):
    if not os.path.exists(folder_path):
        print(f"Folder path '{folder_path}' does not exist.")
        return None
    if not os.path.isdir(folder_path):
        print(f"Path '{folder_path}' is not a directory.")
        return None
    if not extension.startswith('.'):
        extension = '.' + extension
    if not any(f.endswith(extension) for f in os.listdir(folder_path)):
        print(f"No file with extension '{extension}' found in folder '{folder_path}'.")
        return None

    return os.path.join(folder_path, sorted([f for f in os.listdir(folder_path) if f.endswith(extension)])[0])


def get_drone_config(context):
    drone_id = os.getenv("DRONE_ID", "1")  # Default to 1 if not set
    drone_config_path = os.getenv("DRONE_CONFIG_FOLDER_PATH", "./configs")  # Default to current directory if not set
    drone_state_config_path = os.getenv("DRONE_STATE_FOLDER_PATH", "./configs_states")  # Default to current directory if not set

    num_drones = os.getenv("NUM_DRONES", 1)
    num_drones = int(num_drones)
    config_file = get_first_file_with_extension(drone_config_path, '.json')
    state_config_file = get_first_file_with_extension(drone_state_config_path, '.json')

    if not os.path.exists(config_file):
        print(f"Config file '{config_file}' does not exist.")
        return None, None

    return config_file, state_config_file, num_drones, drone_id

def launch_setup(context, *args, **kwargs):
    config_file, state_config_file, num_drones, drone_id = get_drone_config(context)
    print (f"config_file: {config_file}\nstate_config_file: {state_config_file}\nnum_drones: {num_drones}\ndrone_id: {drone_id}")
    if not config_file or not num_drones:
        return []

    # Node from the `controller` package
    controller_node = Node(
        package='controller',
        executable='controller_node',
        name='controller_node',
        output='log',
        parameters=[{'config_file_path': config_file}],
        arguments=['--ros-args', '--log-level', 'debug'],
        log_cmd=True,
    )

    # Node from the `lifecycle_controller` package
    lifecycle_controller_node = Node(
        package='lifecycle_controller',
        executable='lifecycle_controller_node',
        name='lifecycle_controller_node',
        output='log',
        parameters=[{'config_file_path': config_file, 'state_config_file_path':state_config_file, 'num_drones': num_drones}],
        arguments=['--ros-args', '--log-level', 'debug'],
        log_cmd=True,
    )

    nodes_to_launch = [controller_node, lifecycle_controller_node]

    # Optional node `ref_node` launches if `drone_id` is '1'
    if drone_id == "2":
        drone_id = int(drone_id)
        ref_node = Node(
            package='controller',
            executable='ref_point_node',
            name='ref_point_node',
            output='log',
            parameters=[{'config_file_path': config_file, 'num_drones': num_drones, 'drone_id': drone_id}],
            arguments=['--ros-args', '--log-level', 'debug'],
            log_cmd=True,
        )
        nodes_to_launch.append(ref_node)

    return nodes_to_launch
    
def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])