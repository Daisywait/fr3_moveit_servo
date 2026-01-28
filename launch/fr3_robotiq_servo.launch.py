#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition

ROBOT_IP_DEFAULT = "172.16.0.2"
USE_FAKE_HARDWARE_DEFAULT = "false"
FAKE_SENSOR_COMMANDS_DEFAULT = "false"
START_SERVO_DELAY_SEC = 2.0

MOVEIT_CONFIG_PACKAGE = "fr3_robotiq_moveit_config"
ROBOT_DESCRIPTION_PACKAGE = "franka_description"

CONTROL_MODE_DEFAULT = "trajectory"
MOVE_GROUP_NAME_DEFAULT = "fr3_arm"
PLANNING_FRAME_DEFAULT = "fr3_link0"
ROBOT_LINK_COMMAND_FRAME_DEFAULT = "fr3_link0"
EE_FRAME_NAME_DEFAULT = "robotiq_85_base_link"
CHECK_COLLISIONS_DEFAULT = "false"
SCALE_LINEAR_DEFAULT = "1.0"
SCALE_ROTATIONAL_DEFAULT = "1.0"
SCALE_JOINT_DEFAULT = "1.0"

TRAJECTORY_ARM_CONTROLLER = "fr3_arm_controller"
TRAJECTORY_COMMAND_OUT_TOPIC = "/fr3_arm_controller/joint_trajectory"
TRAJECTORY_COMMAND_OUT_TYPE = "trajectory_msgs/JointTrajectory"

VELOCITY_ARM_CONTROLLER = "fr3_velocity_controller"
VELOCITY_COMMAND_OUT_TOPIC = "/fr3_velocity_controller/commands"
VELOCITY_COMMAND_OUT_TYPE = "std_msgs/Float64MultiArray"

URDF_XACRO = "robots/fr3/fr3_robotiq.urdf.xacro"
SRDF_XACRO = "robots/fr3/fr3_robotiq.srdf.xacro"
KINEMATICS_YAML = "config/fr3_robotiq_kinematics.yaml"
ROS2_CONTROLLERS_YAML = "config/fr3_robotiq_ros_controllers.yaml"

def load_yaml(package_name, file_path):
    """从指定包中加载 YAML 文件内容"""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def _declare_launch_args():
    return [
        DeclareLaunchArgument("robot_ip", default_value=ROBOT_IP_DEFAULT),
        DeclareLaunchArgument("use_fake_hardware", default_value=USE_FAKE_HARDWARE_DEFAULT),
        DeclareLaunchArgument("fake_sensor_commands", default_value=FAKE_SENSOR_COMMANDS_DEFAULT),
        DeclareLaunchArgument("control_mode", default_value=CONTROL_MODE_DEFAULT),
        DeclareLaunchArgument("move_group_name", default_value=MOVE_GROUP_NAME_DEFAULT),
        DeclareLaunchArgument("planning_frame", default_value=PLANNING_FRAME_DEFAULT),
        DeclareLaunchArgument(
            "robot_link_command_frame", default_value=ROBOT_LINK_COMMAND_FRAME_DEFAULT
        ),
        DeclareLaunchArgument("ee_frame_name", default_value=EE_FRAME_NAME_DEFAULT),
        DeclareLaunchArgument("check_collisions", default_value=CHECK_COLLISIONS_DEFAULT),
        DeclareLaunchArgument("scale_linear", default_value=SCALE_LINEAR_DEFAULT),
        DeclareLaunchArgument("scale_rotational", default_value=SCALE_ROTATIONAL_DEFAULT),
        DeclareLaunchArgument("scale_joint", default_value=SCALE_JOINT_DEFAULT),
    ]


def _robot_description(robot_ip, use_fake_hardware, fake_sensor_commands):
    urdf_xacro = os.path.join(
        get_package_share_directory(ROBOT_DESCRIPTION_PACKAGE),
        URDF_XACRO,
    )
    robot_description_cmd = Command([
        FindExecutable(name="xacro"), " ",
        urdf_xacro,
        " robot_ip:=", robot_ip,
        " use_fake_hardware:=", use_fake_hardware,
        " fake_sensor_commands:=", fake_sensor_commands,
    ])
    return {"robot_description": ParameterValue(robot_description_cmd, value_type=str)}


def _robot_description_semantic(use_fake_hardware):
    srdf_xacro = os.path.join(
        get_package_share_directory(ROBOT_DESCRIPTION_PACKAGE),
        SRDF_XACRO,
    )
    semantic_cmd = Command([
        FindExecutable(name="xacro"), " ",
        srdf_xacro,
        " use_fake_hardware:=", use_fake_hardware,
    ])
    return {"robot_description_semantic": ParameterValue(semantic_cmd, value_type=str)}


def _robot_description_kinematics():
    kinematics_yaml = load_yaml(MOVEIT_CONFIG_PACKAGE, KINEMATICS_YAML)
    return {"robot_description_kinematics": kinematics_yaml}


def _ros2_control_nodes(robot_description):
    controllers_yaml = os.path.join(
        get_package_share_directory(MOVEIT_CONFIG_PACKAGE),
        ROS2_CONTROLLERS_YAML,
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="screen",
    )
    return robot_state_publisher_node, ros2_control_node


def _spawner_nodes(use_fake_hardware, trajectory_mode_condition, velocity_mode_condition):
    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["robotiq_gripper_controller"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[TRAJECTORY_ARM_CONTROLLER],
            condition=trajectory_mode_condition,
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[VELOCITY_ARM_CONTROLLER],
            condition=velocity_mode_condition,
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["franka_robot_state_broadcaster"],
            condition=UnlessCondition(use_fake_hardware),
            output="screen",
        ),
    ]
    return spawners


def _servo_node(
    robot_description,
    robot_description_semantic,
    robot_description_kinematics,
    command_out_topic,
    command_out_type,
    move_group_name,
    planning_frame,
    robot_link_command_frame,
    ee_frame_name,
    check_collisions,
    scale_linear,
    scale_rotational,
    scale_joint,
    condition,
):
    servo_overrides = {
        "moveit_servo": {
            "command_out_topic": command_out_topic,
            "command_out_type": command_out_type,
            "move_group_name": move_group_name,
            "planning_frame": planning_frame,
            "robot_link_command_frame": robot_link_command_frame,
            "ee_frame_name": ee_frame_name,
            "check_collisions": check_collisions,
            "scale": {
                "linear": scale_linear,
                "rotational": scale_rotational,
                "joint": scale_joint,
            },
        }
    }
    return Node(
        package="moveit_servo",
        executable="servo_node",
        name="moveit_servo",
        output="screen",
        condition=condition,
        parameters=[
            servo_overrides,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )


def _auto_start_servo_action():
    return TimerAction(
        period=START_SERVO_DELAY_SEC,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/moveit_servo/start_servo",
                    "std_srvs/srv/Trigger",
                    "{}",
                ],
                output="screen",
            )
        ],
    )


def generate_launch_description():
    launch_args = _declare_launch_args()

    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    control_mode = LaunchConfiguration("control_mode")
    move_group_name = LaunchConfiguration("move_group_name")
    planning_frame = LaunchConfiguration("planning_frame")
    robot_link_command_frame = LaunchConfiguration("robot_link_command_frame")
    ee_frame_name = LaunchConfiguration("ee_frame_name")
    check_collisions = LaunchConfiguration("check_collisions")
    scale_linear = LaunchConfiguration("scale_linear")
    scale_rotational = LaunchConfiguration("scale_rotational")
    scale_joint = LaunchConfiguration("scale_joint")
    trajectory_mode_condition = IfCondition(
        PythonExpression(["'", control_mode, "' == 'trajectory'"])
    )
    velocity_mode_condition = IfCondition(
        PythonExpression(["'", control_mode, "' == 'velocity'"])
    )

    robot_description = _robot_description(
        robot_ip,
        use_fake_hardware,
        fake_sensor_commands,
    )
    robot_description_semantic = _robot_description_semantic(use_fake_hardware)
    robot_description_kinematics = _robot_description_kinematics()
    robot_state_publisher_node, ros2_control_node = _ros2_control_nodes(robot_description)
    spawners = _spawner_nodes(use_fake_hardware, trajectory_mode_condition, velocity_mode_condition)
    servo_node_trajectory = _servo_node(
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        TRAJECTORY_COMMAND_OUT_TOPIC,
        TRAJECTORY_COMMAND_OUT_TYPE,
        move_group_name,
        planning_frame,
        robot_link_command_frame,
        ee_frame_name,
        check_collisions,
        scale_linear,
        scale_rotational,
        scale_joint,
        trajectory_mode_condition,
    )
    servo_node_velocity = _servo_node(
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        VELOCITY_COMMAND_OUT_TOPIC,
        VELOCITY_COMMAND_OUT_TYPE,
        move_group_name,
        planning_frame,
        robot_link_command_frame,
        ee_frame_name,
        check_collisions,
        scale_linear,
        scale_rotational,
        scale_joint,
        velocity_mode_condition,
    )
    auto_start_servo = _auto_start_servo_action()

    return LaunchDescription([
        *launch_args,
        robot_state_publisher_node,
        ros2_control_node,
        *spawners,
        servo_node_trajectory,
        servo_node_velocity,
        auto_start_servo,
    ])
