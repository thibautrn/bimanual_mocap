#!/usr/bin/env python3
import rclpy, socket, struct, numpy as np
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from rclpy.duration import Duration

UDP_PORT = 50003

def remap_watch_to_base(p):
    x, y, z = map(float, p)
    return np.array([z, -x, y], dtype=float)

class ServoUDPBridge(Node):
    def __init__(self):
        super().__init__("servo_udp_bridge")
        self.pub = self.create_publisher(TwistStamped, "delta_twist_cmds", 10)  # matches ~/delta_twist_cmds
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.01, self.step)  # 100 Hz
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", UDP_PORT))
        self.sock.setblocking(False)

        # target is wrist position (watch → base remap + scaling handled elsewhere if needed)
        self.target_xyz = None
        self.ee_link = "leftarm_wrist_2_link"
        self.base = "base_link"

        # simple gains; keep small for stability
        self.kp_lin = 2.0  # m/s per m error
        self.max_lin = 0.4

        self.get_logger().info(f"Listening UDP on {UDP_PORT}")

    def read_udp(self):
        try:
            data, _ = self.sock.recvfrom(1024)
            if len(data) < 28:  # need at least wrist (x,y,z) floats after rotations
                return
            # your packet layout used (wrist rot[4], wrist pos[3], ...). We only need wrist pos here
            # hand_rot = struct.unpack('ffff', data[0:16])
            hand_pos = struct.unpack('fff',  data[16:28])
            wr = remap_watch_to_base(hand_pos)
            self.target_xyz = wr  # already in base frame
        except BlockingIOError:
            pass

    def current_wrist_in_base(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.base, self.ee_link, Time(), timeout=Duration(seconds=0.0))
            p = tf.transform.translation
            return np.array([p.x, p.y, p.z], dtype=float)
        except Exception as e:
            self.get_logger().warn(f"TF err: {e}")
            return None

    def step(self):
        self.read_udp()
        if self.target_xyz is None:
            return
        cur = self.current_wrist_in_base()
        if cur is None:
            return

        err = self.target_xyz - cur
        v = self.kp_lin * err
        speed = np.linalg.norm(v)
        if speed > self.max_lin:
            v *= self.max_lin / (speed + 1e-9)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base          # IMPORTANT: matches robot_link_command_frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = v.tolist()
        # keep angular zeros — wrists are excluded via active_subgroup, so shoulder+elbow solve the motion
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ServoUDPBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
