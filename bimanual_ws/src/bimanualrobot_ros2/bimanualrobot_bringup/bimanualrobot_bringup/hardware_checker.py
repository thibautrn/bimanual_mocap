#!/usr/bin/env python3
import rclpy
import yaml
import math
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dynamixel_sdk import PortHandler, PacketHandler

# Control table address
ADDR_PRESENT_POSITION = 611  # Address for Present Position

class HardwareChecker(Node):
    """
    Checks Dynamixel hardware connections, reads motor positions,
    and publishes them as joint states.
    """
    def __init__(self):
        super().__init__('hardware_checker')

        # Declare and get the path to the YAML config file
        self.declare_parameter('config_file', '')
        config_file_path = self.get_parameter('config_file').get_parameter_value().string_value

        if not config_file_path:
            self.get_logger().error("'config_file' parameter not set. Please provide the path to the hardware config YAML.")
            return

        # Load and parse the YAML file
        try:
            with open(config_file_path, 'r') as file:
                self.config = yaml.safe_load(file)
        except Exception as e:
            self.get_logger().error(f"Failed to load or parse YAML file {config_file_path}: {e}")
            return

        self.arm_configs = self._extract_arm_configs()
        self.port_handlers = {}
        self.packet_handler = PacketHandler(2.0)

        # Initialize hardware and check connections
        if not self.initialize_hardware():
            self.get_logger().error("Failed to initialize hardware. Shutting down.")
            self.destroy_node()
            return

        # Create a publisher for the /joint_states topic
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        # Timer to periodically read and publish joint states
        self.timer = self.create_timer(0.1, self.read_and_publish_states) # 10 Hz
        self.get_logger().info("Hardware checker started. Publishing to /joint_states...")

    def _extract_arm_configs(self):
        """Extracts arm configurations from the loaded YAML."""
        configs = {}
        # Correctly look for the hardware configuration, not the controller configuration
        for key, value in self.config.items():
            if key.endswith('_dynamixel_hardware'):
                try:
                    params = value['ros__parameters']
                    # Derive arm name from the hardware key, e.g., 'left_arm_dynamixel_hardware' -> 'left_arm'
                    arm_name = key.replace('_dynamixel_hardware', '')
                    
                    # The YAML stores joint names in a list and joint IDs under their own keys.
                    # We need to iterate through the list of names and look up the ID for each one.
                    joint_map = {}
                    for joint_name in params['joints']:
                        if joint_name in params and 'id' in params[joint_name]:
                            joint_map[joint_name] = params[joint_name]['id']
                        else:
                            self.get_logger().warn(f"No ID found for joint '{joint_name}' in '{key}' config.")

                    configs[arm_name] = {
                        'port': params['usb_port'],
                        'baud_rate': params['baud_rate'],
                        'joints': joint_map
                    }
                    self.get_logger().info(f"Found configuration for '{arm_name}'")
                except KeyError as e:
                    self.get_logger().warn(f"Could not parse config for '{key}': Missing key {e}")
        return configs

    def initialize_hardware(self):
        """Initializes ports, keeps them open, and pings each Dynamixel."""
        self.get_logger().info("--- Initializing Hardware ---")
        all_motors_ok = True
        for arm, config in self.arm_configs.items():
            port_name = config['port']
            if port_name not in self.port_handlers:
                port_handler = PortHandler(port_name)
                if not port_handler.openPort():
                    self.get_logger().error(f"[{arm}] Failed to open port {port_name}")
                    all_motors_ok = False
                    continue
                if not port_handler.setBaudRate(config['baud_rate']):
                    self.get_logger().error(f"[{arm}] Failed to set baud rate on {port_name}")
                    all_motors_ok = False
                    continue
                self.port_handlers[port_name] = port_handler
                self.get_logger().info(f"[{arm}] Port {port_name} opened successfully.")

            self.get_logger().info(f"[{arm}] Pinging motors...")
            for joint_name, joint_id in config['joints'].items():
                model, result, error = self.packet_handler.ping(self.port_handlers[port_name], joint_id)
                if result != 0 or error != 0:
                    self.get_logger().error(f"  - Joint '{joint_name}' (ID: {joint_id}): FAILED")
                    all_motors_ok = False
                else:
                    self.get_logger().info(f"  + Joint '{joint_name}' (ID: {joint_id}): SUCCESS. Model: {model}")
        
        self.get_logger().info("--- Hardware Initialization Complete ---")
        return all_motors_ok

    def read_and_publish_states(self):
        """Reads the current position of all motors and publishes as a JointState message."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for arm, config in self.arm_configs.items():
            port_name = config['port']
            port_handler = self.port_handlers.get(port_name)
            if not port_handler:
                continue

            for joint_name, joint_id in config['joints'].items():
                pos, result, error = self.packet_handler.read4ByteTxRx(port_handler, joint_id, ADDR_PRESENT_POSITION)
                if result == 0 and error == 0:
                    # Convert Dynamixel position (0-4095) to radians
                    # Assumes 0 position is at 2048 ticks
                    angle_rad = (pos - 2048) * (2 * math.pi) / 4096
                    msg.name.append(joint_name)
                    msg.position.append(angle_rad)
                else:
                    self.get_logger().warn(f"Could not read position for {joint_name} (ID: {joint_id})")
        
        self.joint_state_publisher.publish(msg)

    def shutdown_hook(self):
        """Closes all open serial ports."""
        self.get_logger().info("Closing serial ports.")
        for port_handler in self.port_handlers.values():
            port_handler.closePort()

def main(args=None):
    rclpy.init(args=args)
    node = HardwareChecker()
    try:
        if rclpy.ok():
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_hook()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()