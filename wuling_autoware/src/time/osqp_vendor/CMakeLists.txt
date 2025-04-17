cmake_minimum_required(VERSION 3.9)

project(osqp_vendor)

find_package(ros_environment REQUIRED)

if($ENV{ROS_VERSION} EQUAL 1)
  find_package(catkin REQUIRED)
elseif($ENV{ROS_VERSION} EQUAL 2)
  find_package(ament_cmake REQUIRED)
endif()

macro(build_osqp)
  set(local_osqp_path "${CMAKE_SOURCE_DIR}/osqp-0.6.2")  # 设置本地路径
  set(extra_cmake_args)

  get_property(multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
  if(NOT multi_config AND DEFINED CMAKE_BUILD_TYPE)
    list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
  endif()

  include(ExternalProject)
  externalproject_add(osqp_local
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/osqp-v0.6.2  # 设置构建的临时目录
    SOURCE_DIR ${local_osqp_path}  # 使用本地 OSQP 源码路径
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/osqp_install  # 设置安装路径
      ${extra_cmake_args}
    BUILD_ALWAYS 1  # 总是构建，即使本地已有源码
  )

  # The external project will install to the build folder, but we'll install that on make install.
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/osqp_install/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}
    USE_SOURCE_PERMISSIONS
    PATTERN config.h EXCLUDE
  )
endmacro()

build_osqp()

if($ENV{ROS_VERSION} EQUAL 1)
  catkin_package(
    LIBRARIES osqp
    DEPENDS osqp
    CFG_EXTRAS osqp_vendor-extras.cmake)
elseif($ENV{ROS_VERSION} EQUAL 2)
  ament_export_libraries(osqp)
  ament_export_dependencies(osqp)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package(CONFIG_EXTRAS cmake/osqp_vendor-extras.cmake)
endif()
