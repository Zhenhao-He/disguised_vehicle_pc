cmake_minimum_required(VERSION 3.14)
project(autoware_signal_processing)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(lowpass_filters SHARED
  src/lowpass_filter_1d.cpp
  src/lowpass_filter.cpp
  src/butterworth.cpp)



ament_auto_package()
