cmake_minimum_required(VERSION 3.14.4)
project(filters)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

##############################################################################
# Find dependencies
##############################################################################

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost QUIET COMPONENTS headers)
if(NOT Boost_headers_FOUND)
  find_package(Boost REQUIRED)
endif()

##############################################################################
# Build
##############################################################################


if("${CMAKE_VERSION}" VERSION_GREATER_EQUAL 3.19)
  add_library(realtime_circular_buffer
    INTERFACE
      include/filters/realtime_circular_buffer.h
      include/filters/realtime_circular_buffer.hpp
  )
else()
  add_library(realtime_circular_buffer INTERFACE)
endif()

target_link_libraries(realtime_circular_buffer
  INTERFACE
    Boost::boost
)

# filter_base is an INTERFACE library because it only has header files.
# It is not compiled, but sets up the following properties for consumers
# to link to a target.
# For more info, see the CMake Wiki on INTERFACE targets.
# https://cmake.org/cmake/help/latest/command/add_library.html#interface-libraries


if("${CMAKE_VERSION}" VERSION_GREATER_EQUAL 3.19)
  add_library(filter_base
    INTERFACE
      include/filters/filter_base.h
      include/filters/filter_base.hpp
  )
else()
  add_library(filter_base INTERFACE)
endif()

target_link_libraries(filter_base
  INTERFACE
    rclcpp::rclcpp
)

if("${CMAKE_VERSION}" VERSION_GREATER_EQUAL 3.19)
  add_library(filter_chain
    INTERFACE
      include/filters/filter_chain.h
      include/filters/filter_chain.hpp
  )
else()
  add_library(filter_chain INTERFACE)
endif()


target_link_libraries(filter_chain
  INTERFACE
    filter_base
    pluginlib::pluginlib
)

set(interface_targets realtime_circular_buffer filter_base filter_chain)
foreach(target ${interface_targets})
  target_include_directories(${target}
    INTERFACE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
endforeach()



# Plugins
add_library(mean SHARED src/mean.cpp)
target_link_libraries(mean
  PUBLIC
    realtime_circular_buffer
    filter_base
    pluginlib::pluginlib
)

add_library(params SHARED src/test_params.cpp)
target_link_libraries(params
  PUBLIC
    filter_base
    pluginlib::pluginlib
)

add_library(increment SHARED src/increment.cpp)
target_link_libraries(increment
  PUBLIC
    realtime_circular_buffer
    filter_base
    pluginlib::pluginlib
)

add_library(median SHARED src/median.cpp)
target_link_libraries(median
  PUBLIC
    realtime_circular_buffer
    filter_base
    pluginlib::pluginlib
)

add_library(transfer_function SHARED src/transfer_function.cpp)
target_link_libraries(transfer_function
  PUBLIC
    realtime_circular_buffer
    filter_base
    pluginlib::pluginlib
)

set(plugin_targets mean params increment median transfer_function)
foreach(target ${plugin_targets})
  target_include_directories(${target}
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
endforeach()

install(TARGETS ${interface_targets} ${plugin_targets}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})


##############################################################################
# Install
##############################################################################

# Export old-style CMake variables for includes
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  Boost
  pluginlib
  rclcpp
)

# Install pluginlib xml
pluginlib_export_plugin_description_file(filters "default_plugins.xml")
ament_package()
