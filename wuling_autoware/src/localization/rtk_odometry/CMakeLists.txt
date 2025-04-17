
#（1）设置版本和工程
cmake_minimum_required(VERSION 3.8)
project(rtk_odometry)

#（2）设置C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#（3）设置自动查找，根据package.xml读取
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#（4）添加非ROS标准包和外部库


#（5）添加可执行文件
ament_auto_add_executable(${PROJECT_NAME}_node 
  src/rtk_odometry_node.cpp
)

#（6）#如果头文件不在include或src文件里，设置特定头文件目录hpp_folders，目标设置为可执行文件${PROJECT_NAME}_node 


#（7）#链接非ROS标准包和外部库，目标设置为可执行文件${PROJECT_NAME}_node 


#（8）添加测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#（9）launch和config安装到share
ament_auto_package(
  INSTALL_TO_SHARE
    launch
)