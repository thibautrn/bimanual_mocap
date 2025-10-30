#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, RobotState
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
import time

class BimanualDrumstickClient(Node):
    def __init__(self):
        super().__init__('bimanual_drumstick_client')
        self.declare_parameter('cycle_speed', 2.0)  # seconds per move
        self._action_client_right = ActionClient(self, MoveGroup, 'move_action')
        self._action_client_left = ActionClient(self, MoveGroup, 'move_action')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)

    def get_tip_pose(self, tip_link):
        
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link', tip_link, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.0)
            )
            return trans.transform.translation, trans.transform.rotation
        except Exception as e:
            self.get_logger().error(f"TF lookup failed for {tip_link}: {e}")
            return None, None

    def send_goal(self, group_name, tip_link, target_pos, target_ori, action_client):
        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = group_name
        goal_msg.request.num_planning_attempts = 1
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.start_state = RobotState()

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "base_link"
        position_constraint.link_name = tip_link
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = target_pos.x
        pose.pose.position.y = target_pos.y
        pose.pose.position.z = target_pos.z
        pose.pose.orientation = target_ori
        position_constraint.constraint_region.primitives.append(sphere)
        position_constraint.constraint_region.primitive_poses.append(pose.pose)

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "base_link"
        orientation_constraint.link_name = tip_link
        orientation_constraint.orientation = target_ori
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        goal_msg.request.goal_constraints.append(constraints)

        action_client.wait_for_server()
        self.get_logger().info(f"Sending goal for {tip_link} to z={target_pos.z:.2f} ...")
        future = action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result.result.error_code.val == 1:
            self.get_logger().info(f'Motion succeeded for {tip_link}!')
        else:
            self.get_logger().error(f'Motion failed for {tip_link}! Error code: {result.result.error_code.val}')

def main():
    rclpy.init()
    node = BimanualDrumstickClient()

    cycle_speed = node.get_parameter('cycle_speed').get_parameter_value().double_value

    # Get initial poses
    pos_right, ori_right = node.get_tip_pose('rightarm_drumstick_tip')
    pos_left, ori_left = node.get_tip_pose('leftarm_drumstick_tip')
    if pos_right is None or ori_right is None or pos_left is None or ori_left is None:
        node.destroy_node()
        rclpy.shutdown()
        return

    for i in range(6):  # 3 cycles up and down
        # Move both tips up
        pos_right.z += 0.10
        pos_left.z += 0.10
        node.send_goal('rightarm', 'rightarm_drumstick_tip', pos_right, ori_right, node._action_client_right)
        node.send_goal('leftarm', 'leftarm_drumstick_tip', pos_left, ori_left, node._action_client_left)
        time.sleep(cycle_speed)

        # Move both tips down
        pos_right.z -= 0.10
        pos_left.z -= 0.10
        node.send_goal('rightarm', 'rightarm_drumstick_tip', pos_right, ori_right, node._action_client_right)
        node.send_goal('leftarm', 'leftarm_drumstick_tip', pos_left, ori_left, node._action_client_left)
        time.sleep(cycle_speed)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()