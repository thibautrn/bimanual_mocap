from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch file to run the joint state checker."""
    
    # Define the joint names for each arm. 
    # Replace these with the actual joint names from your robot's configuration.
    left_joint_names = [
        'left_joint_1', 'left_joint_2', 'left_joint_3', 
        'left_joint_4', 'left_joint_5', 'left_joint_6'
    ]
    
    right_joint_names = [
        'right_joint_1', 'right_joint_2', 'right_joint_3', 
        'right_joint_4', 'right_joint_5', 'right_joint_6'
    ]

    joint_state_checker_node = Node(
        package='bimanualrobot_bringup',
        executable='joint_state_checker',
        name='joint_state_checker',
        output='screen',
        parameters=[{
            'left_joint_names': left_joint_names,
            'right_joint_names': right_joint_names
        }]
    )

    return LaunchDescription([
        joint_state_checker_node
    ])