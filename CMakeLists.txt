cmake_minimum_required(VERSION 3.8)
project(fr3_moveit_servo)

find_package(ament_cmake REQUIRED)

# 只是装 launch 和 config，不需要编译代码
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()

