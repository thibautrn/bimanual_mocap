#!/bin/bash
# Single script to launch the bimanualrobot with Gazebo, RViz, and MoveIt 2

cleanup() {
    echo "Cleaning up..."
    sleep 5.0
    pkill -9 -f "ros2|gazebo|gz|nav2|amcl|bt_navigator|nav_to_pose|rviz2|assisted_teleop|cmd_vel_relay|robot_state_publisher|joint_state_publisher|move_to_free|mqtt|autodock|cliff_detection|moveit|move_group|basic_navigator"
}

# Set up cleanup trap
trap 'cleanup' SIGINT SIGTERM

echo "Launching Gazebo simulation..."
ros2 launch bimanualrobot_gazebo bimanualrobot.gazebo.launch.py \
    load_controllers:=true \
    world_file:=empty.world \
    use_camera:=true \
    use_rviz:=false \
    use_robot_state_pub:=true \
    use_sim_time:=true \
    x:=0.0 \
    y:=0.0 \
    z:=0.0 \
    roll:=0.0 \
    pitch:=0.0 \
    yaw:=0.0 &

sleep 15

echo "Spawning velocity controller..."
ros2 run controller_manager spawner joint_group_velocity_controller --controller-manager /controller_manager &

echo "Launching MoveIt..."
ros2 launch bimanualrobot_moveit_config move_group.launch.py &

# Wait for move_group to be ready
echo "Waiting for move_group..."
until ros2 node list | grep -q "^/move_group$"; do sleep 1; done

echo "Adjusting camera position..."
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 2000 --req "pose: {position: {x: 1.36, y: 0.0, z: 0.95} orientation: {x: 0, y: 0, z: 3.14, w: 0}}"

# Keep the script running until Ctrl+C
wait