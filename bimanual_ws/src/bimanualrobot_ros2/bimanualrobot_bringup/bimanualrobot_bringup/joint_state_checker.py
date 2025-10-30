import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading

class JointStateChecker(Node):
    """Node to subscribe to joint states and print them."""

    def __init__(self):
        super().__init__('joint_state_checker')

        # Declare parameters for joint names for both arms
        self.declare_parameter('left_joint_names', [''])
        self.declare_parameter('right_joint_names', [''])

        self.left_joint_names = self.get_parameter('left_joint_names').get_parameter_value().string_array_value
        self.right_joint_names = self.get_parameter('right_joint_names').get_parameter_value().string_array_value

        if not self.left_joint_names or not self.left_joint_names[0]:
            self.get_logger().error("Parameter 'left_joint_names' not set or empty.")
            return
        if not self.right_joint_names or not self.right_joint_names[0]:
            self.get_logger().error("Parameter 'right_joint_names' not set or empty.")
            return

        self.joint_states = {}
        self.lock = threading.Lock()

        # Subscribe to the /joint_states topic
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        # Create a timer to print the joint states periodically
        self.timer = self.create_timer(2.0, self.print_joint_states)
        self.get_logger().info("Joint State Checker started. Waiting for joint states...")

    def joint_state_callback(self, msg):
        """Callback to update the current joint states."""
        with self.lock:
            for i, name in enumerate(msg.name):
                self.joint_states[name] = msg.position[i]

    def print_joint_states(self):
        """Prints the current states of the joints for each arm."""
        with self.lock:
            if not self.joint_states:
                self.get_logger().warn("No joint states received yet...")
                return

            self.get_logger().info("---------------------------------")
            self.get_logger().info("Current Joint States:")
            
            self.get_logger().info("--- Left Arm ---")
            for name in self.left_joint_names:
                position = self.joint_states.get(name, 'N/A')
                if isinstance(position, float):
                    self.get_logger().info(f"  {name}: {position:.4f}")
                else:
                    self.get_logger().warn(f"  {name}: {position}")

            self.get_logger().info("--- Right Arm ---")
            for name in self.right_joint_names:
                position = self.joint_states.get(name, 'N/A')
                if isinstance(position, float):
                    self.get_logger().info(f"  {name}: {position:.4f}")
                else:
                    self.get_logger().warn(f"  {name}: {position}")
            self.get_logger().info("---------------------------------")


def main(args=None):
    rclpy.init(args=args)
    node = JointStateChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()