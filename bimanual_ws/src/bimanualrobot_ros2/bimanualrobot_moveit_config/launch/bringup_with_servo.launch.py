# bimanualrobot_moveit_config/launch/bringup_with_servo.launch.py
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro, yaml, os

def _load_xml(pkg, rel):
    p = os.path.join(get_package_share_directory(pkg), rel)
    assert os.path.exists(p), f"Missing file: {p}"
    with open(p, 'r') as f:
        return f.read()

def _load_yaml(pkg, rel):
    p = os.path.join(get_package_share_directory(pkg), rel)
    assert os.path.exists(p), f"Missing file: {p}"
    with open(p, 'r') as f:
        return yaml.safe_load(f)

def _load_xacro(pkg, rel, mappings=None):
    p = os.path.join(get_package_share_directory(pkg), rel)
    assert os.path.exists(p), f"Missing file: {p}"
    doc = xacro.process_file(p, mappings=mappings or {})
    return doc.toxml()

def generate_launch_description():
    # ---- EXACT paths in your install layout ----
    urdf_rel = "urdf/robots/bimanualrobot.urdf.xacro"               # bimanualrobot_description
    srdf_rel = "config/bimanualrobot/bimanualrobot.srdf"            # bimanualrobot_moveit_config
    kin_rel  = "config/bimanualrobot/kinematics.yaml"
    jl_rel   = "config/bimanualrobot/joint_limits.yaml"
    servo_rel= "config/bimanualrobot/servo_params.yaml"

    robot_description_xml = _load_xacro("bimanualrobot_description", urdf_rel)
    robot_description_semantic_xml = _load_xml("bimanualrobot_moveit_config", srdf_rel)
    kinematics_yaml  = _load_yaml("bimanualrobot_moveit_config", kin_rel)
    joint_limits_yaml= _load_yaml("bimanualrobot_moveit_config", jl_rel)
    servo_yaml       = _load_yaml("bimanualrobot_moveit_config", servo_rel)

    params = [
        {"robot_description": robot_description_xml},
        {"robot_description_semantic": robot_description_semantic_xml},
        {"robot_description_kinematics": kinematics_yaml},
        {"robot_description_planning": joint_limits_yaml},
        {"moveit_servo": servo_yaml["moveit_servo"]},  # ensure nested key
    ]

    servo = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_server",   # was "servo_server"
        namespace="left",
        parameters=params,
    )

    return LaunchDescription([servo])
