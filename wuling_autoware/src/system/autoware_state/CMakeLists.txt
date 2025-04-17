#（1）设置版本和工程
cmake_minimum_required(VERSION 3.8)
project(autoware_state)

#（2）设置C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#（3）设置自动查找，根据package.xml读取
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#（5）添加共享库，并根据共享库注册节点
ament_auto_add_library(${PROJECT_NAME} SHARED 
  src/autoware_state.cpp
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "default_ad_api::AutowareStateNode"
  EXECUTABLE autoware_state_exe
)




#（9）launch和config安装到share
ament_auto_package(
  INSTALL_TO_SHARE
    launch 
)
