cmake_minimum_required(VERSION 3.8)
project(position_transform)

# 默认使用C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)  # 必须添加这个包

# 添加可执行文件
add_executable(position_transform 
  src/position_transform.cpp
)

# 添加依赖
ament_target_dependencies(position_transform
  rclcpp
  tf2_ros
  tf2
  geometry_msgs
  tf2_geometry_msgs  # 必须添加这个依赖
)

# 安装
install(TARGETS
position_transform
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()