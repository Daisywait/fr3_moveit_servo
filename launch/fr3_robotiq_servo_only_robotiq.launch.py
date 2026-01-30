#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import UnlessCondition

ROBOT_IP_DEFAULT = "172.16.0.2"
USE_FAKE_HARDWARE_DEFAULT = "false"
FAKE_SENSOR_COMMANDS_DEFAULT = "false"

MOVEIT_CONFIG_PACKAGE = "fr3_robotiq_moveit_config"
ROBOT_DESCRIPTION_PACKAGE = "franka_description"

URDF_XACRO = "robots/fr3/fr3_robotiq.urdf.xacro"
ROS2_CONTROLLERS_YAML = "config/robotiq_only_ros_controllers.yaml"


def _declare_launch_args():
    return [
        DeclareLaunchArgument("robot_ip", default_value=ROBOT_IP_DEFAULT),
        DeclareLaunchArgument("use_fake_hardware", default_value=USE_FAKE_HARDWARE_DEFAULT),
        DeclareLaunchArgument("fake_sensor_commands", default_value=FAKE_SENSOR_COMMANDS_DEFAULT),
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


def _ros2_control_node(robot_description):
    controllers_yaml = os.path.join(
        os.path.dirname(__file__),
        "..",
        "config",
        "robotiq_only_ros_controllers.yaml",
    )
    return Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="screen",
    )


def _spawner_nodes():
    return [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["robotiq_gripper_controller"],
            output="screen",
        ),
    ]


def generate_launch_description():
    launch_args = _declare_launch_args()

    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")

    robot_description = _robot_description(
        robot_ip,
        use_fake_hardware,
        fake_sensor_commands,
    )
    ros2_control_node = _ros2_control_node(robot_description)
    spawners = _spawner_nodes()

    return LaunchDescription([
        *launch_args,
        ros2_control_node,
        *spawners,
    ])
