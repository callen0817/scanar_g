cmake_minimum_required(VERSION 3.8)
project(viture_hud)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  src/hud_overlay.py
  src/hud_renderer.py
  src/hud_state_manager.py
  src/product_capabilities.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY src/hud_widgets
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
