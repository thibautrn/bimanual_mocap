#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity

class DrumSpawner(Node):
    def __init__(self):
        super().__init__('drum_spawner')
        self.cli = self.create_client(SpawnEntity, '/world/default/create')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /world/default/create service...')
        self.spawn_drum()

    def spawn_drum(self):
        req = SpawnEntity.Request()
        req.sdf_filename = "/home/mrelm/GitHub/Bimanual_Robot/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_gazebo/models/drum_cylinder/model.sdf"
        req.name = "drum_cylinder"
        req.pose.position.x = 0.4
        req.pose.position.y = 0.0
        req.pose.position.z = 0.98
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info('Drum spawned successfully!')
        else:
            self.get_logger().error('Failed to spawn drum.')

def main(args=None):
    rclpy.init(args=args)
    node = DrumSpawner()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()