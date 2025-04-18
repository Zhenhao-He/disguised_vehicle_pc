# Generated from: mrt_cmake_modules/cmake/mrt_cmake_modules-extra.cmake.in
if(_MRT_CMAKE_MODULES_EXTRAS_INCLUDED_)
    return()
endif()
set(_MRT_CMAKE_MODULES_EXTRAS_INCLUDED_ TRUE)

# Check cmakelists version
set(_MRT_RECOMMENDED_VERSION 4.0)
if(MRT_PKG_VERSION AND MRT_PKG_VERSION VERSION_LESS _MRT_RECOMMENDED_VERSION )
   message(WARNING "Current CMakeLists.txt version is less than the recommended version ${_MRT_RECOMMENDED_VERSION}. If you are the maintainer, please update it with\n'mrt maintenance update_cmakelists ${PROJECT_NAME}'.")
endif()

# Set the cmake install path
if()
    # cmake dir in develspace
    set(MRT_CMAKE_MODULES_ROOT_PATH "/home/parking/hezhenhao/test/src/mrt_cmake_modules")
    set(MRT_CMAKE_MODULES_CMAKE_PATH "/home/parking/hezhenhao/test/src/mrt_cmake_modules/cmake")
else()
    # cmake dir in installspace
    set(MRT_CMAKE_MODULES_ROOT_PATH "${mrt_cmake_modules_DIR}/..")
    set(MRT_CMAKE_MODULES_CMAKE_PATH "${mrt_cmake_modules_DIR}")
endif()

list(APPEND CMAKE_MODULE_PATH "${MRT_CMAKE_MODULES_CMAKE_PATH}/Modules")
set(MCM_TEMPLATE_DIR "${MRT_CMAKE_MODULES_CMAKE_PATH}/Templates")
include(${MRT_CMAKE_MODULES_CMAKE_PATH}/mrt_cmake_modules-macros.cmake)
