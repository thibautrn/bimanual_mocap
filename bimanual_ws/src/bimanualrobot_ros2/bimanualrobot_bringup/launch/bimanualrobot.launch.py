#!/usr/bin/env python3

# Import necessary modules for launch description and actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare all launch arguments that can be set when launching this file
    declared_arguments = [
        DeclareLaunchArgument(
            'start_rviz', default_value='false',
            description='Whether to execute rviz2'),  # Option to start RViz visualization
        DeclareLaunchArgument(
            'init_position', default_value='true', 
            description='Whether to launch the init_position node'),  # Option to initialize robot position
        DeclareLaunchArgument(
            'init_position_file', default_value='initial_positions.yaml', 
            description='Path to the initial position file'),  # File for initial joint positions
        DeclareLaunchArgument('prefix', default_value='',
                          description='Prefix for robot joints and links'),  # Prefix for joint/link names
        DeclareLaunchArgument('add_world', default_value='true',
                            choices=['true', 'false'],
                            description='Whether to add world link'),  # Option to add a world link
        DeclareLaunchArgument('base_link', default_value='base_link',
                            description='Name of the base link'),  # Name of the robot's base link
        DeclareLaunchArgument('gripper_type', default_value='fixed_gripper',
                            description='Type of the gripper'),  # Type of gripper to use
        DeclareLaunchArgument('use_gazebo', default_value='false',
                            choices=['true', 'false'],
                            description='Whether to use Gazebo simulation'),  # Option for Gazebo simulation
        DeclareLaunchArgument('use_gripper', default_value='false',
                            choices=['true', 'false'],
                            description='Whether to attach a gripper')  # Option to attach a gripper
    ]

    # Create launch configuration objects for each argument to use in substitutions
    start_rviz = LaunchConfiguration('start_rviz')  # Whether to start RViz
    prefix = LaunchConfiguration('prefix')  # Prefix for joints/links
    use_gazebo = LaunchConfiguration('use_gazebo')  # Whether to use Gazebo
    add_world = LaunchConfiguration('add_world')  # Whether to add world link
    base_link = LaunchConfiguration('base_link')  # Name of base link
    gripper_type = LaunchConfiguration('gripper_type')  # Type of gripper
    use_gripper = LaunchConfiguration('use_gripper')  # Whether to attach gripper
    init_position = LaunchConfiguration('init_position')  # Whether to initialize position
    init_position_file = LaunchConfiguration('init_position_file')  # Initial position file

    # Generate the robot description (URDF) using xacro and the provided arguments
    urdf_file = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),  # Find the xacro executable
        ' ',
        PathJoinSubstitution([
            FindPackageShare('bimanualrobot_description'),  # Find the package containing the robot description
            'urdf',
            'robots',
            'bimanualrobot.urdf.xacro',  # Path to the main xacro file
        ]),
        ' ',
        'prefix:=', prefix,  # Pass prefix argument to xacro
        ' ',
        'use_gazebo:=', use_gazebo,  # Pass use_gazebo argument to xacro
        ' ',
        'add_world:=', add_world,  # Pass add_world argument to xacro
        ' ',
        'gripper_type:=', gripper_type,  # Pass gripper_type argument to xacro
        ' ',
        'use_gripper:=', use_gripper,  # Pass use_gripper argument to xacro
        ' ',
        'base_link:=', base_link  # Pass base_link argument to xacro
    ])

    # Path to the controller manager YAML configuration file
    controller_manager_config = PathJoinSubstitution([
        FindPackageShare('bimanualrobot_bringup'),  # Find the bringup package
        'config',
        'hardware_controller_manager.yaml',  # YAML file with hardware/controller parameters
    ])

    test_yaml_config = PathJoinSubstitution([
        FindPackageShare('bimanualrobot_bringup'),  # Find the bringup package
        'config',
        'test.yaml',  # YAML file with test hardware/controller parameters
    ])

    # Path to the RViz configuration file
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('bimanualrobot_description'),  # Find the robot description package
        'rviz',
        'bimanualrobot.rviz',  # RViz config file
    ])

    # Path to the trajectory parameters file for initial positions
    trajectory_params_file = PathJoinSubstitution([
        FindPackageShare('bimanualrobot_bringup'),  # Find the bringup package
        'config',
        init_position_file,  # Initial position YAML file
    ])

    # Node to start the ros2_control controller manager (hardware interface)
    control_node = Node(
        package='controller_manager',  # ROS 2 package for controller manager
        executable='ros2_control_node',  # Executable for ros2_control
        parameters=[
            {'robot_description': ParameterValue(urdf_file, value_type=str)},  # Pass robot description parameter
            controller_manager_config  # Pass controller manager config YAML
        ],
        output='both',  # Output logs to both screen and log file
        condition=UnlessCondition(use_gazebo),  # Only start if not using Gazebo simulation
    )

    # Node to publish the robot's state (TF and joint states)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',  # ROS 2 package for state publisher
        executable='robot_state_publisher',  # Executable for state publisher
        parameters=[{'robot_description': ParameterValue(urdf_file, value_type=str), 'use_sim_time': use_gazebo}],  # Pass robot description and sim time
        output='both',  # Output logs to both screen and log file
    )

    # Node to execute joint trajectories for initialization
    joint_trajectory_executor = Node(
        package='bimanualrobot_bringup',  # Custom package for trajectory execution
        executable='joint_trajectory_executor',  # Executable for trajectory executor
        parameters=[trajectory_params_file],  # Pass trajectory parameters file
        output='both',  # Output logs to both screen and log file
        condition=IfCondition(init_position),  # Only start if init_position is true
    )

    # Node to start RViz for visualization
    rviz_node = Node(
        package='rviz2',  # ROS 2 package for RViz
        executable='rviz2',  # Executable for RViz
        arguments=['-d', rviz_config_file],  # Pass RViz config file
        output='both',  # Output logs to both screen and log file
        condition=IfCondition(start_rviz),  # Only start if start_rviz is true
    )

    # Node to execute joint trajectories for left arm initialization
    left_joint_trajectory_executor = Node(
        package='bimanualrobot_bringup',  # Package containing the executor
        executable='joint_trajectory_executor',  # Executable name
        name='left_joint_trajectory_executor',  # Unique node name
        parameters=[trajectory_params_file, {'use_arm': 'left'}],  # Use left arm config
        output='both',
        condition=IfCondition(init_position),
    )

    # Node to execute joint trajectories for right arm initialization
    right_joint_trajectory_executor = Node(
        package='bimanualrobot_bringup',
        executable='joint_trajectory_executor',
        name='right_joint_trajectory_executor',
        parameters=[trajectory_params_file, {'use_arm': 'right'}],  # Use right arm config
        output='both',
        condition=IfCondition(init_position),
    )

    # Spawn joint_state_broadcaster first
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'],
        output='both',
        parameters=[{'robot_description': ParameterValue(urdf_file, value_type=str)}],
        condition=UnlessCondition(use_gazebo),
    )

    # Then left arm
    left_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_controller', '--controller-manager-timeout', '60'],
        output='both',
        parameters=[{'robot_description': ParameterValue(urdf_file, value_type=str)}],
        condition=UnlessCondition(use_gazebo),
    )

    # Then right arm
    right_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_arm_controller', '--controller-manager-timeout', '60'],
        output='both',
        parameters=[{'robot_description': ParameterValue(urdf_file, value_type=str)}],
        condition=UnlessCondition(use_gazebo),
    )

    delay_left_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[left_controller_spawner],
        )
    )

    delay_right_after_left = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_controller_spawner,
            on_exit=[right_controller_spawner],
        )
    )

    # Start both executors only after right controller spawner exits
    delay_executors_after_right = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_controller_spawner,
            on_exit=[left_joint_trajectory_executor, right_joint_trajectory_executor],
        )
    )

    # Event handler to delay RViz startup until controllers are spawned
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,  # Wait for controller spawner to exit
            on_exit=[rviz_node]  # Then start RViz
        )
    )

    # Event handler to delay joint trajectory executor until controllers are spawned
    # delay_joint_trajectory_executor_after_controllers = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=robot_controller_spawner,  # Wait for controller spawner to exit
    #         on_exit=[joint_trajectory_executor],  # Then start trajectory executor
    #     )
    # )

    # # Event handler to delay joint trajectory executors until controllers are spawned
    # delay_joint_trajectory_executors_after_controllers = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=robot_controller_spawner,
    #         on_exit=[left_joint_trajectory_executor, right_joint_trajectory_executor],
    #     )
    # )

    # Return the complete launch description with all declared arguments and nodes
    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            robot_state_publisher_node,
            jsb_spawner,
            delay_left_after_jsb,
            delay_right_after_left,
            delay_executors_after_right,
            delay_rviz_after_joint_state_broadcaster_spawner,
        ]
    )