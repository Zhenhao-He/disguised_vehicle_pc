cmake_minimum_required(VERSION 3.5)
project(coordinate_transform)

# 找到所有需要的包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# 添加源代码目录
add_executable(coordinate_transform_node src/coordinate_transform_node.cpp)

# 链接相关的库
ament_target_dependencies(coordinate_transform_node
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
  nav_msgs
)

# 链接 OpenCV 库
target_link_libraries(coordinate_transform_node ${OpenCV_LIBS})

# 安装
install(TARGETS
  coordinate_transform_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
