from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
import os

def launch_setup(context, *args, **kwargs):

    # Node from the `controller` package
    target_node = Node(
        package='target',  # Ensure this is the correct package name
        executable='target_node',
        name='target_node',
        output='log',
        parameters=[],
        arguments=['--ros-args', '--log-level', 'debug'],
        log_cmd=True,  # Enables logging of the command
    )

    nodes_to_launch = [target_node]
    return nodes_to_launch
 
def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])