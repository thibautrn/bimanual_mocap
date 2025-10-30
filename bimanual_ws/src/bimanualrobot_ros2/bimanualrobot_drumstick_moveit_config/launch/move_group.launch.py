#!/usr/bin/env python3
"""
Launch MoveIt 2 for the bimanualrobot robotic arm.

This script creates a ROS 2 launch file that starts the necessary nodes and services
for controlling a bimanualrobot robotic arm using MoveIt 2. It loads configuration files,
starts the move_group node, and optionally launches RViz for visualization.

:author: Mohamed El Mistiri
:date: Aug 17, 2025
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, OpaqueFunction, LogInfo
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """
    Generate a launch description for MoveIt 2 with bimanualrobot robot.

    This function sets up the necessary configuration and nodes to launch MoveIt 2
    for controlling a bimanualrobot robotic arm. It includes setting up paths to config files,
    declaring launch arguments, configuring the move_group node, and optionally starting RViz.

    Returns:
        LaunchDescription: A complete launch description for the MoveIt 2 system
    """
    # Constants for paths to different files and folders
    package_name_moveit_config = 'bimanualrobot_drumstick_moveit_config'

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    rviz_config_package = LaunchConfiguration('rviz_config_package')

    # Get the package share directory for moveit config
    pkg_share_moveit_config_temp = FindPackageShare(package=package_name_moveit_config)

    # Declare the launch arguments for flexibility and clarity
    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value='bimanualrobot',
        description='Name of the robot to use')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RViz')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value='move_group.rviz',
        description='RViz configuration file')

    declare_rviz_config_package_cmd = DeclareLaunchArgument(
        name='rviz_config_package',
        default_value=package_name_moveit_config,
        description='Package containing the RViz configuration file')

    def configure_setup(context):
        """Configure MoveIt and create nodes with proper string conversions."""
        # Log the start of MoveIt configuration
        yield LogInfo(msg="Starting MoveIt 2 launch configuration for bimanualrobot...")

        # Get the robot name as a string for use in MoveItConfigsBuilder
        robot_name_str = LaunchConfiguration('robot_name').perform(context)
        yield LogInfo(msg=f"Robot name set to: {robot_name_str}")

        # Get package path
        pkg_share_moveit_config = pkg_share_moveit_config_temp.find(package_name_moveit_config)
        yield LogInfo(msg=f"MoveIt config package path: {pkg_share_moveit_config}")

        # Construct file paths using robot name string
        config_path = os.path.join(pkg_share_moveit_config, 'config', robot_name_str)
        yield LogInfo(msg=f"Config path: {config_path}")

        # Define all config file paths
        initial_positions_file_path = os.path.join(config_path, 'initial_positions.yaml')
        joint_limits_file_path = os.path.join(config_path, 'joint_limits.yaml')
        kinematics_file_path = os.path.join(config_path, 'kinematics.yaml')
        moveit_controllers_file_path = os.path.join(config_path, 'moveit_controllers.yaml')
        srdf_model_path = os.path.join(config_path, f'{robot_name_str}.srdf')
        pilz_cartesian_limits_file_path = os.path.join(config_path, 'pilz_cartesian_limits.yaml')

        # Log the config files being used
        yield LogInfo(msg=f"SRDF model path: {srdf_model_path}")
        yield LogInfo(msg=f"MoveIt controllers config: {moveit_controllers_file_path}")

        # Create MoveIt configuration using MoveItConfigsBuilder
        moveit_config = (
            MoveItConfigsBuilder(robot_name_str, package_name=package_name_moveit_config)
            .trajectory_execution(file_path=moveit_controllers_file_path)
            .robot_description(
                mappings={
                    "use_gripper": "false",
                }
            )
            .robot_description_semantic(file_path=srdf_model_path)
            .joint_limits(file_path=joint_limits_file_path)
            .robot_description_kinematics(file_path=kinematics_file_path)
            .planning_pipelines(
                pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"],
                default_planning_pipeline="ompl"
            )
            .planning_scene_monitor(
                publish_robot_description=False,
                publish_robot_description_semantic=True,
                publish_planning_scene=True,
            )
            .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file_path)
            .to_moveit_configs()
        )
        yield LogInfo(msg="MoveIt configuration built successfully.")

        # MoveIt capabilities dictionary
        move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}
        yield LogInfo(msg="Move group capabilities set.")

        # Create move_group node for planning and execution
        start_move_group_node_cmd = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {'use_sim_time': use_sim_time},
                # {'start_state': {'content': initial_positions_file_path}},
                {'start_state_max_bounds_error': 0.02},
                move_group_capabilities,
            ],
        )
        yield LogInfo(msg="move_group node created.")

        # Create RViz node for visualization (if requested)
        start_rviz_node_cmd = Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d",
                [FindPackageShare(rviz_config_package), "/config/", rviz_config_file]
            ],
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                {'use_sim_time': use_sim_time}
            ],
        )
        yield LogInfo(msg="RViz node created (if enabled).")

        # RViz exit handler to shutdown launch when RViz is closed
        exit_event_handler = RegisterEventHandler(
            condition=IfCondition(use_rviz),
            event_handler=OnProcessExit(
                target_action=start_rviz_node_cmd,
                on_exit=EmitEvent(event=Shutdown(reason='rviz exited')),
            ),
        )
        yield LogInfo(msg="RViz exit event handler registered.")

        # Return all nodes and handlers to be launched
        yield start_move_group_node_cmd
        yield start_rviz_node_cmd
        yield exit_event_handler

    # Create the launch description object
    ld = LaunchDescription()

    # Add the launch arguments for flexibility
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_rviz_config_package_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # Add the setup and node creation (with logging)
    ld.add_action(OpaqueFunction(function=configure_setup))

    return ld