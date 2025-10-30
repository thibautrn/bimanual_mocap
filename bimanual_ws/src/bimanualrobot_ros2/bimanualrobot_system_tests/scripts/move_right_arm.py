#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, RobotState
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
import time
import threading, socket, struct
import numpy as np

# ============================================================
# UDP listener globals
hand_pos = None
larm_pos = None
uarm_pos = None
hand_rot = None
larm_rot = None
uarm_rot = None
hips_rot = None

# ============================================================
def udp_listener(port=50003):
    """Background thread: listens for UDP pose data and updates globals."""
    global hand_rot, hand_pos, larm_rot, larm_pos, uarm_rot, uarm_pos, hips_rot
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    print(f"[UDP] Listening on port {port} ...")

    while True:
        try:
            data, _ = sock.recvfrom(1024)
            if len(data) < 100:
                continue

            hand_rot = struct.unpack('ffff', data[0:16])
            hand_pos = struct.unpack('fff',  data[16:28])
            larm_rot = struct.unpack('ffff', data[28:44])
            larm_pos = struct.unpack('fff',  data[44:56])
            uarm_rot = struct.unpack('ffff', data[56:72])
            uarm_pos = struct.unpack('fff',  data[72:84])
            hips_rot = struct.unpack('ffff', data[84:100])

        except Exception as e:
            print(f"[UDP ERROR] {e}")
            continue

# ============================================================
# MoveIt Client
class MoveLeftArmClient(Node):
    def __init__(self):
        super().__init__('move_left_drumstick_tip_client')
        self.declare_parameter('cycle_speed', 0)  # seconds per move
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Spin to allow TF to populate
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)

    def get_tip_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link', 'leftarm_ee_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.0)
            )
            return trans.transform.translation, trans.transform.rotation
        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return None, None

    def send_goal(self, target_pos, target_ori):
        goal_msg = MoveGroup.Goal()
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = "leftarm"
        goal_msg.request.num_planning_attempts = 1
        goal_msg.request.allowed_planning_time = 1.0
        goal_msg.request.start_state = RobotState()

        # Position constraint (1 cm sphere around target_pos)
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "base_link"
        position_constraint.link_name = "leftarm_ee_link"
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]  # radius [m]
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = target_pos.x
        pose.pose.position.y = target_pos.y
        pose.pose.position.z = target_pos.z
        pose.pose.orientation = target_ori  # orientation of region; irrelevant for a sphere
        position_constraint.constraint_region.primitives.append(sphere)
        position_constraint.constraint_region.primitive_poses.append(pose.pose)

        # Orientation constraint on EE link
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "base_link"
        orientation_constraint.link_name = "leftarm_ee_link"
        orientation_constraint.orientation = target_ori
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        goal_msg.request.goal_constraints.append(constraints)

        self._action_client.wait_for_server()
        self.get_logger().info(f"Sending goal to ({target_pos.x:.3f}, {target_pos.y:.3f}, {target_pos.z:.3f}) ...")
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result.result.error_code.val == 1:
            self.get_logger().info('Motion succeeded!')
        else:
            self.get_logger().error(f'Motion failed! Error code: {result.result.error_code.val}')

import numpy as np
# ============================================================
# Helpers
SHOULDER_ANCHOR = np.array([0.04, 0.32, 1.526])
L1, L2 = 0.26, 0.39

def remap_watch_to_base(p):
    x,y,z = map(float, p)
    return np.array([z, -x, y], dtype=float)

def scale_watch_to_robot(uarm, larm, hand):
    # Remap to robot base frame
    sh = remap_watch_to_base(uarm)
    el = remap_watch_to_base(larm)
    wr = remap_watch_to_base(hand)

    # Segment unit vectors
    v1 = el - sh
    v2 = wr - el
    u1 = v1 / (np.linalg.norm(v1) + 1e-9)
    u2 = v2 / (np.linalg.norm(v2) + 1e-9)

    # Scale to robot lengths, anchored at SHOULDER_ANCHOR
    sh_robot = SHOULDER_ANCHOR
    el_robot = sh_robot + L1 * u1
    wr_robot = el_robot + L2 * u2
    return sh_robot, el_robot, wr_robot

def quat_from_matrix(R):
    """Convert a 3x3 rotation matrix to (x,y,z,w) quaternion. Assumes R is a valid rotation matrix."""
    qw = np.sqrt(1 + R[0,0] + R[1,1] + R[2,2]) / 2
    qx = (R[2,1] - R[1,2]) / (4*qw)
    qy = (R[0,2] - R[2,0]) / (4*qw)
    qz = (R[1,0] - R[0,1]) / (4*qw)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def quat_from_arm(shoulder, elbow, wrist):
    S = np.array(shoulder)
    E = np.array(elbow)
    W = np.array(wrist)

    z = W - E
    z /= np.linalg.norm(z)

    x = E - S
    x = x - np.dot(x,z)*z   # project out component along z
    x /= np.linalg.norm(x)

    y = np.cross(z, x)

    R = np.column_stack((x,y,z))  # 3x3 rotation
    return quat_from_matrix(R)
# ============================================================
def main():
    rclpy.init()
    node = MoveLeftArmClient()
    cycle_speed = node.get_parameter('cycle_speed').get_parameter_value().double_value

    while rclpy.ok():
        if hand_pos is None or larm_pos is None or uarm_pos is None:
            time.sleep(0.05)
            continue
        sh_robot, el_robot, wr_robot = scale_watch_to_robot(uarm_pos,larm_pos,hand_pos)
        q_ros = quat_from_arm(sh_robot,el_robot,wr_robot)

        pos, ori = node.get_tip_pose()
        pos_goal = type(pos)()
        pos_goal.x = wr_robot[0]
        pos_goal.y = wr_robot[1]
        pos_goal.z = wr_robot[2]


        node.send_goal(pos_goal, q_ros)
        time.sleep(cycle_speed)

    node.destroy_node()
    rclpy.shutdown()


def main2():
    # --- Your smartwatch pose (watch frame) ---
    rclpy.init()
    node = MoveLeftArmClient()

    # Query a current pose (not strictly required, but keeps same API)
    pos, ori = node.get_tip_pose()
    if pos is None or ori is None:
        node.destroy_node()
        rclpy.shutdown()
        return

    # Build target geometry_msgs fields
    pos_goal = type(pos)()  # same type as TF translation (geometry_msgs/Vector3)
    sh_robot, el_robot, wr_robot = scale_watch_to_robot(uarm_pos,larm_pos,hand_pos)

    pos_goal.x, pos_goal.y, pos_goal.z = wr_robot

    q_ros = quat_from_arm(sh_robot,el_robot,wr_robot)


    # Send single goal
    cycle_speed = node.get_parameter('cycle_speed').get_parameter_value().double_value
    node.send_goal(pos_goal, q_ros)
    time.sleep(cycle_speed)

    node.destroy_node()
    rclpy.shutdown()


# ============================================================
if __name__ == "__main__":
    udp_thread = threading.Thread(target=udp_listener, daemon=True)
    udp_thread.start()
    main()
