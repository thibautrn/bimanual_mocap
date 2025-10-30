#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class DrumMarkerPublisher(Node):
    def __init__(self):
        super().__init__('drum_marker_publisher')
        pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.publish_drum(pub)

    def publish_drum(self, pub):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.ns = "drum"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = 0.17
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.98
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.155  # diameter = 2*radius
        marker.scale.y = 0.155
        marker.scale.z = 0.045  # length
        marker.color.r = 0.6
        marker.color.g = 0.3
        marker.color.b = 0.1
        marker.color.a = 1.0
        pub.publish(marker)
        self.get_logger().info("Published drum marker.")

def main(args=None):
    rclpy.init(args=args)
    node = DrumMarkerPublisher()
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()