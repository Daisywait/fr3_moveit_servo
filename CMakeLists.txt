cmake_minimum_required(VERSION 3.8)
project(fr3_moveit_servo)

find_package(ament_cmake REQUIRED)

# 安装 launch 和 config
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
