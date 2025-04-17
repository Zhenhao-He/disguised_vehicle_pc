# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_mrt_cmake_modules_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED mrt_cmake_modules_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(mrt_cmake_modules_FOUND FALSE)
  elseif(NOT mrt_cmake_modules_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(mrt_cmake_modules_FOUND FALSE)
  endif()
  return()
endif()
set(_mrt_cmake_modules_CONFIG_INCLUDED TRUE)

# output package information
if(NOT mrt_cmake_modules_FIND_QUIETLY)
  message(STATUS "Found mrt_cmake_modules: 1.0.11 (${mrt_cmake_modules_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'mrt_cmake_modules' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${mrt_cmake_modules_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(mrt_cmake_modules_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "mrt_cmake_modules-extras.cmake")
foreach(_extra ${_extras})
  include("${mrt_cmake_modules_DIR}/${_extra}")
endforeach()
