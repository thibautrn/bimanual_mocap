#!/usr/bin/env python3
import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dynamixel_sdk import PortHandler, PacketHandler

class HardwareCheckerPing(Node):
    """
    Checks Dynamixel hardware connections based on a YAML config file
    and reports joint states.
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

        self.joint_states = {}
        self.arm_configs = self._extract_arm_configs()

        # Check hardware connections
        self.check_hardware_connections()

        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Timer to periodically print status
        self.timer = self.create_timer(3.0, self.print_status)
        self.get_logger().info("Hardware Checker started. Subscribing to /joint_states...")

    def _extract_arm_configs(self):
        """Extracts arm configurations from the loaded YAML."""
        configs = {}
        # Assuming the parameters are nested under a hardware name
        # that ends with 'arm_dynamixel_hardware'
        for key, value in self.config.items():
            if key.endswith('arm_dynamixel_hardware'):
                try:
                    params = value['ros__parameters']
                    arm_name = key.replace('_dynamixel_hardware', '')
                    
                    # Extract joint names and their corresponding IDs
                    joint_info = {}
                    for joint_name in params['joints']:
                        if joint_name in params:
                            joint_info[joint_name] = params[joint_name]['id']
                        else:
                            self.get_logger().warn(f"Details for joint '{joint_name}' not found in config for '{key}'")

                    configs[arm_name] = {
                        'port': params['usb_port'],
                        'baud_rate': params['baud_rate'],
                        'joints': joint_info
                    }

                    self.get_logger().info(f"Found configuration for '{arm_name}'")
                except KeyError as e:
                    self.get_logger().warn(f"Could not parse config for '{key}': Missing key {e}")
        return configs

    def check_hardware_connections(self):
        """Initializes Port/Packet handlers and pings each Dynamixel."""
        self.get_logger().info("--- Checking Hardware Connections ---")
        packet_handler = PacketHandler(2.0) # Protocol 2.0

        for arm, config in self.arm_configs.items():
            port_handler = PortHandler(config['port'])
            self.get_logger().info(f"[{arm}] Checking port: {config['port']} at {config['baud_rate']} baud.")
            
            # Open port
            if not port_handler.openPort():
                self.get_logger().error(f"[{arm}] Failed to open port {config['port']}")
                continue
            
            # Set baud rate
            if not port_handler.setBaudRate(config['baud_rate']):
                self.get_logger().error(f"[{arm}] Failed to set baud rate to {config['baud_rate']}")
                port_handler.closePort()
                continue

            self.get_logger().info(f"[{arm}] Port opened successfully. Pinging motors...")
            for joint_name, joint_id in config['joints'].items():
                # Ping the motor
                model_number, dxl_comm_result, dxl_error = packet_handler.ping(port_handler, joint_id)
                if dxl_comm_result != 0:
                    self.get_logger().error(f"  - Joint '{joint_name}' (ID: {joint_id}): FAILED to communicate. Result: {packet_handler.getTxRxResult(dxl_comm_result)}")
                elif dxl_error != 0:
                    self.get_logger().error(f"  - Joint '{joint_name}' (ID: {joint_id}): FAILED with error. Error: {packet_handler.getRxPacketError(dxl_error)}")
                else:
                    self.get_logger().info(f"  + Joint '{joint_name}' (ID: {joint_id}): SUCCESS. Model: {model_number}")
            
            port_handler.closePort()
        self.get_logger().info("--- Hardware Check Complete ---")

    def joint_state_callback(self, msg):
        """Callback to update the current joint states."""
        for i, name in enumerate(msg.name):
            if msg.position:
                self.joint_states[name] = msg.position[i]

    def print_status(self):
        """Prints the current states of the joints for each arm."""
        if not self.joint_states:
            self.get_logger().warn("No joint states received yet from /joint_states topic...")
            return

        self.get_logger().info("--- Current Joint States ---")
        for arm, config in self.arm_configs.items():
            self.get_logger().info(f"--- {arm} ---")
            for joint_name in config['joints'].keys():
                position = self.joint_states.get(joint_name)
                if position is not None:
                    self.get_logger().info(f"  {joint_name}: {position:.4f} rad")
                else:
                    self.get_logger().warn(f"  {joint_name}: State not available")
        self.get_logger().info("----------------------------")


def main(args=None):
    rclpy.init(args=args)
    node = HardwareChecker()
    if node.config:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()