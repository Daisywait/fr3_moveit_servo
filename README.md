# fr3_moveit_servo

## 概述

`fr3_moveit_servo` 是一个 ROS 2 功能包，为 Franka Emika FR3 机器人搭配 Robotiq 夹爪提供 MoveIt Servo 的配置和启动文件。

## 功能特性

- **实时运动控制**：通过 MoveIt Servo 实现对 FR3 机器人的实时运动控制
- **碰撞检测**：集成 MoveIt 的碰撞检测功能，确保安全操作
- **灵活配置**：提供详细的参数配置，可根据需求调整控制参数
- **真机与仿真支持**：支持真实硬件和仿真环境（通过 `use_fake_hardware` 参数切换）

## 包结构

```
fr3_moveit_servo/
├── config/
│   └── (removed)                   # Servo 参数已内置在 launch 中
├── launch/
│   ├── fr3_robotiq_servo.launch.py # 主启动文件（twist 伺服）
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 依赖项

### 必需依赖

- `rclcpp` - ROS 2 C++ 客户端库
- `moveit_servo` - MoveIt Servo 功能包
- `moveit_configs_utils` - MoveIt 配置工具
- `franka_fr3_moveit_config` - FR3 MoveIt 配置包
- `ament_index_python` - ROS 2 Python 包索引
- `python3-yaml` - YAML 解析库

## 配置文件说明

### Servo 参数配置

该文件包含了 MoveIt Servo 的核心配置参数：

- **移动组配置**：
  - `move_group_name`: 设置为 "fr3_arm"
  - `planning_frame`: 规划坐标系 "fr3_link0"
  - `ee_frame_name`: 末端执行器坐标系 "robotiq_85_base_link"

- **控制参数**：
  - `command_in_type`: 输入命令类型为 "twist"
  - `command_out_type`: 输出类型为 "trajectory_msgs/JointTrajectory"
  - `publish_period`: 发布周期 0.01 秒（100 Hz）

- **缩放因子**：
  - `linear`: 线速度缩放因子 0.5
  - `rotational`: 角速度缩放因子 0.5
  - `joint`: 关节速度缩放因子 0.8

- **碰撞检测**：
  - `check_collisions`: 启用碰撞检测
  - `collision_check_rate`: 碰撞检测频率 10.0 Hz

## 启动文件说明

### launch/fr3_robotiq_servo.launch.py

该启动文件的作用是**一次性启动运行 MoveIt Servo 所需的全部系统组件**，主要包括：

#### 1. 加载机器人模型和配置
- **URDF 加载**：加载机器人物理描述（从 `franka_description` 包）
- **SRDF 加载**：加载语义描述文件（规划组、碰撞矩阵等）
- **运动学配置**：加载逆运动学求解器参数

#### 2. 启动 ROS 2 Control 系统
- **robot_state_publisher**：发布机器人状态到 TF 树
- **ros2_control_node**：ROS 2 Control 管理器节点
- **控制器启动**：
  - `joint_state_broadcaster`：发布关节状态
  - `robotiq_gripper_controller`：夹爪控制器
  - `fr3_arm_controller`：机械臂轨迹控制器
  - `franka_robot_state_broadcaster`：Franka 机器人状态广播器（仅真机）

#### 3. 启动 MoveIt Servo 节点
- 配置并启动 `moveit_servo` 节点
- 通过 launch 参数配置 Servo（见下方示例）
- 将机器人描述、语义描述和运动学配置传递给 Servo 节点

#### 4. 提供灵活的启动参数
- `robot_ip`：机器人 IP 地址（真机使用，默认: "172.16.0.2"）
- `use_fake_hardware`：是否使用仿真硬件（默认: "false"）
- `fake_sensor_commands`：是否使用假传感器命令（默认: "false"）
- `command_out_topic`：命令输出话题（可配置）

**总结**：启动文件将机器人模型加载、控制器启动、MoveIt Servo 启动等多个步骤整合在一起，使得整个系统可以通过一条命令启动并运行，大大简化了使用流程。

## 使用方法

### 1. 编译功能包

在工作空间中编译该功能包：

```bash
cd ~/your_ws
colcon build --packages-select fr3_moveit_servo
source install/setup.bash
```

### 2. 启动 MoveIt Servo

#### 使用真机

```bash
ros2 launch fr3_moveit_servo fr3_robotiq_servo.launch.py robot_ip:=172.16.0.2
```

#### 使用仿真

```bash
ros2 launch fr3_moveit_servo fr3_robotiq_servo.launch.py use_fake_hardware:=true
```

### 3. 启动参数

- `robot_ip` (默认: "172.16.0.2"): 机器人 IP 地址（仅真机需要）
- `use_fake_hardware` (默认: "false"): 是否使用仿真硬件
- `fake_sensor_commands` (默认: "false"): 是否使用假传感器命令
- `command_out_topic` (默认: "/fr3_arm_controller/joint_trajectory"): 输出命令话题

### 4. 控制机器人

启动后，MoveIt Servo 会订阅以下话题来接收控制命令：

- 输入话题：`/moveit_servo/delta_twist_cmds` (geometry_msgs/TwistStamped)

发布关节轨迹到：

- 输出话题：`/fr3_arm_controller/joint_trajectory` (trajectory_msgs/JointTrajectory)

## 配置说明

### 调试模式（关节速度控制）

当前配置使用关节轨迹控制。如需调试，可以临时切换到关节速度控制（仅用于调试，非成熟的控制模式）：

```yaml
# 关节轨迹控制（当前配置，推荐使用）
command_out_type: "trajectory_msgs/JointTrajectory"
command_out_topic: "/fr3_arm_controller/joint_trajectory"

# 关节速度控制（仅用于调试）
# command_out_type: "std_msgs/Float64MultiArray"
# command_out_topic: "/fr3_velocity_controller/commands"
```

### 调整控制参数

在 launch 参数中可以调整：

- **缩放因子** (`scale`): 调整线速度、角速度和关节速度的缩放比例
- **发布频率** (`publish_period`): 调整命令发布频率
- **碰撞检测** (`check_collisions`, `collision_check_rate`): 配置碰撞检测参数

## 注意事项

1. **网络配置**：使用真机时，确保机器人 IP 地址配置正确且网络连通
2. **安全操作**：首次使用时建议降低缩放因子，确保机器人运动安全
3. **碰撞检测**：虽然启用了碰撞检测，但在实际使用中仍需谨慎操作
4. **控制器配置**：确保对应的 ROS 2 控制器已正确配置并运行

## 维护者

- 维护者: shuangmulin
- 许可证: BSD-3-Clause

## 许可证

本功能包遵循 BSD-3-Clause 许可证。
