cmake_minimum_required(VERSION 3.8)
project(template_middleware)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_auto_package()
