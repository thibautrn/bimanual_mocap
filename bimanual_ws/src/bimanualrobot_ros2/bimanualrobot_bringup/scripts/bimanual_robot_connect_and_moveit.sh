#!/bin/bash
# Single script to launch the bimanualrobot with Gazebo, RViz, and MoveIt 2

cleanup() {
    echo "Cleaning up..."
    sleep 5.0
    pkill -9 -f "ros2|gazebo|gz|nav2|amcl|bt_navigator|nav_to_pose|rviz2|assisted_teleop|cmd_vel_relay|robot_state_publisher|joint_state_publisher|move_to_free|mqtt|autodock|cliff_detection|moveit|move_group|basic_navigator"
}

# Set up cleanup trap
trap 'cleanup' SIGINT SIGTERM

echo "Launching Real Robot..."
ros2 launch bimanualrobot_bringup bimanualrobot.launch.py \
    start_rviz:=false \
    init_position:=true \
    add_world:=true \
    base_link:='base_link' \
    gripper_type:='fixed_gripper' \
    use_gazebo:=false \
    use_gripper:=false &

sleep 15
ros2 launch bimanualrobot_moveit_config move_group.launch.py \
    use_sim_time:=false \
    use_rviz:=true &

# Keep the script running until Ctrl+C


wait