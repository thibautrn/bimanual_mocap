import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch file to run the hardware checker."""
    
    # Get the path to the config file
    config_file = os.path.join(
        get_package_share_directory('bimanualrobot_bringup'),
        'config',
        'hardware_controller_manager.yaml'
    )

    hardware_checker_node = Node(
        package='bimanualrobot_bringup',
        executable='hardware_checker',
        name='hardware_checker',
        output='screen',
        parameters=[{'config_file': config_file}]
    )

    return LaunchDescription([
        hardware_checker_node
    ])