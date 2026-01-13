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

def load_yaml(package_name, file_path):
    """从指定包中加载 YAML 文件内容"""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # ========= Launch 参数声明 (command_out_topic 默认指向速度控制器) =========
    robot_ip_arg = DeclareLaunchArgument("robot_ip", default_value="172.16.0.2")
    use_fake_hardware_arg = DeclareLaunchArgument("use_fake_hardware", default_value="false") # 默认设为 false
    fake_sensor_commands_arg = DeclareLaunchArgument("fake_sensor_commands", default_value="false")
    
    # 将输出话题默认设置为速度控制器的话题
    #command_topic_default = "/fr3_arm_controller/joint_trajectory"
    command_topic_default = "/fr3_arm_controller/joint_trajectory"
    #command_topic_default = "/fr3_velocity_controller/commands"

    command_topic_arg = DeclareLaunchArgument("command_out_topic", default_value=command_topic_default)

    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    command_out_topic = LaunchConfiguration("command_out_topic")
    
    new_moveit_package = "fr3_robotiq_moveit_config"
    robot_description_package = "franka_description"
    
    # ========= 1. URDF/SRDF/Kinematics 加载 =========
    
    # URDF (robot_description)
    fr3_urdf_xacro = os.path.join(
        get_package_share_directory(robot_description_package),
        "robots/fr3/fr3_robotiq.urdf.xacro"
    )

    robot_description_cmd = Command([
        FindExecutable(name="xacro"), " ",
        fr3_urdf_xacro,
        " robot_ip:=", robot_ip,
        " use_fake_hardware:=", use_fake_hardware,
        " fake_sensor_commands:=", fake_sensor_commands,
    ])

    robot_description = {"robot_description": ParameterValue(robot_description_cmd, value_type=str)}

    # SRDF (robot_description_semantic)
    fr3_srdf_xacro = os.path.join(
        get_package_share_directory(robot_description_package),
        "robots/fr3/fr3_robotiq.srdf.xacro"
    )
    robot_description_semantic_cmd = Command([
        FindExecutable(name="xacro"), " ",
        fr3_srdf_xacro,
        " use_fake_hardware:=", use_fake_hardware,
    ])
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_cmd, value_type=str)}

    # Kinematics
    kinematics_yaml = load_yaml(new_moveit_package, "config/fr3_robotiq_kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # ========= 2. ROS 2 Control 节点 =========
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    ros2_controllers_yaml = os.path.join(
        get_package_share_directory(new_moveit_package),
        "config",
        "fr3_robotiq_ros_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description,ros2_controllers_yaml,],
        output="screen",
    )

    # ========= 3. 控制器 Spawner (关键修改: 自动启动速度控制器) =========
    spawners = [
        Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"], output="screen"),

        # 启动夹爪控制器
        Node(package="controller_manager", executable="spawner", arguments=["robotiq_gripper_controller"], output="screen"),


        # 速度控制器
        #Node(package="controller_manager", executable="spawner", arguments=["fr3_velocity_controller", "-c","/controller_manager","--activate"], output="screen"),

        #备选：
        #Node(package="controller_manager", executable="spawner", arguments=["fr3_position_controller", "-c", "/controller_manager", "--activate"], output="screen"),

        #轨迹控制器
        Node(package="controller_manager", executable="spawner", arguments=["fr3_arm_controller"], output="screen"),
    ]

    # 真机才 spawn franka_robot_state_broadcaster
    spawners.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["franka_robot_state_broadcaster"],
            condition=UnlessCondition(use_fake_hardware),
            output="screen",
        )
    )

    # ========= 4. MoveIt Servo 节点 (关键修改: 参数重写) =========
    servo_yaml_file = os.path.join(
        get_package_share_directory("fr3_moveit_servo"),
        "config",
        "fr3_robotiq_servo.yaml",
    )
 
# Force override Servo parameters
    servo_override_params = {
        "moveit_servo": {
            # ========================= 核心修复 =========================
            # 1. 告诉节点不要检查 Twist 类型，防止崩溃
            "command_in_type": "unitless",

            # 2. 【关键】强制指定 Pose 订阅话题！
            # 只有加上这行，Subscription count 才会变成 1
            "pose_command_in_topic": "/moveit_servo/pose_target_cmds",
            
            # 3. 将原来的 Cartesian (Twist) 话题移开，避免占位
            "cartesian_command_in_topic": "/moveit_servo/delta_twist_cmds",
            # ===========================================================

            "move_group_name": "fr3_arm",
            "ee_frame_name": "robotiq_85_base_link",
            "planning_frame": "fr3_link0",
            "robot_link_command_frame": "fr3_link0",

            # 输出配置
            "command_out_topic": command_out_topic,
            "command_out_type": "trajectory_msgs/JointTrajectory",

            "publish_period": 0.01,
            "low_latency_mode": False,
            "incoming_command_timeout": 1.0, # 宽容的时间戳

            # 运动参数
            "linear_scale": 0.8,
            "rotational_scale": 0.5,
            
            # 碰撞检测 (建议调试时先关掉)
            "check_collisions": False,
        }
    }
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="moveit_servo",
        output="screen",
        parameters=[
            servo_yaml_file,
            servo_override_params,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        # 参数
        robot_ip_arg,
        use_fake_hardware_arg,
        fake_sensor_commands_arg,
        command_topic_arg,
        
        # 节点
        robot_state_publisher_node,
        ros2_control_node,
        *spawners,
        servo_node,
    ])