cmake_minimum_required(VERSION 3.5)

project(mimick_vendor)

find_package(ament_cmake REQUIRED)

macro(build_mimick)
  set(local_mimick_path "${CMAKE_SOURCE_DIR}/Mimick-ros2")  # 设置本地路径
  set(extra_cmake_args)

  get_property(multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
  if(NOT multi_config AND DEFINED CMAKE_BUILD_TYPE)
    list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
  endif()

  include(ExternalProject)
  externalproject_add(mimick_local
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/mimick-v0.6.2  # 设置构建的临时目录
    SOURCE_DIR ${local_mimick_path}  # 使用本地 mimick 源码路径
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/mimick_install  # 设置安装路径
      ${extra_cmake_args}
    BUILD_ALWAYS 1  # 总是构建，即使本地已有源码
  )

  # The external project will install to the build folder, but we'll install that on make install.
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/mimick_install/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}
    USE_SOURCE_PERMISSIONS
    PATTERN config.h EXCLUDE
  )
endmacro()

build_mimick()
ament_export_include_directories(include)
ament_export_libraries(mimick)
ament_export_dependencies(mimick)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# this ensures that the package has an environment hook setting the PATH
ament_package()
