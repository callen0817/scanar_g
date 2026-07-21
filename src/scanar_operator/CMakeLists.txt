cmake_minimum_required(VERSION 3.8)
project(scanar_operator)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  src/input_manager.py
  src/session_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
