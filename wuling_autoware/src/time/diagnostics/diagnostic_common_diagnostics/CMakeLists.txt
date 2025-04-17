cmake_minimum_required(VERSION 3.5)
project(diagnostic_common_diagnostics)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
    ${PROJECT_NAME}/cpu_monitor.py
    ${PROJECT_NAME}/ntp_monitor.py
    ${PROJECT_NAME}/ram_monitor.py
    ${PROJECT_NAME}/sensors_monitor.py
    ${PROJECT_NAME}/hd_monitor.py
    DESTINATION lib/${PROJECT_NAME}
)



ament_package()
