cmake_minimum_required(VERSION 3.8)
project(scanar_diagnostics_dashboard)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  src/dashboard_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
