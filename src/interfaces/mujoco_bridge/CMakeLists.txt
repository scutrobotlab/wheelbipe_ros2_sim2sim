cmake_minimum_required(VERSION 3.8)
project(mujoco_ros2_control)

# Use one language level across the complete workspace.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)
find_package(joint_limits REQUIRED)
find_package(urdf REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)

set(THIS_PACKAGE_DEPENDS
  ament_cmake
  rclcpp
  hardware_interface
  pluginlib
  controller_manager
  joint_limits
  urdf
  glfw3
  control_toolbox
  sensor_msgs
  std_msgs
  rosgraph_msgs
)
if(DEFINED ENV{MUJOCO_DIR} AND NOT "$ENV{MUJOCO_DIR}" STREQUAL "")
  set(_MUJOCO_DIR "$ENV{MUJOCO_DIR}")
else()
  # bootstrap.sh installs the pinned dependency here. An explicit MUJOCO_DIR
  # remains the only override, so unrelated system installations cannot be
  # selected accidentally.
  get_filename_component(_PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
  set(_MUJOCO_DIR "${_PROJECT_ROOT}/.deps/mujoco-3.5.0")
endif()

message(STATUS "Using MuJoCo from ${_MUJOCO_DIR}")
unset(MUJOCO_LIB CACHE)
unset(MUJOCO_INCLUDE_DIR CACHE)
find_library(
  MUJOCO_LIB
  NAMES mujoco
  HINTS "${_MUJOCO_DIR}/lib" "${_MUJOCO_DIR}/build/lib"
  NO_DEFAULT_PATH)
find_path(
  MUJOCO_INCLUDE_DIR
  NAMES mujoco/mujoco.h
  HINTS "${_MUJOCO_DIR}/include"
  NO_DEFAULT_PATH)

if(NOT MUJOCO_LIB OR NOT MUJOCO_INCLUDE_DIR)
  message(FATAL_ERROR "Failed to find MuJoCo. Source setup_mujoco_env.bash or set MUJOCO_DIR to the MuJoCo install prefix.")
endif()

if(NOT EXISTS "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h")
  message(FATAL_ERROR "MuJoCo header not found: ${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h")
endif()

add_library(mujoco_system_plugins SHARED src/mujoco_system.cpp)
ament_target_dependencies(mujoco_system_plugins ${THIS_PACKAGE_DEPENDS})
target_link_libraries(mujoco_system_plugins ${MUJOCO_LIB})
target_include_directories(mujoco_system_plugins
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${MUJOCO_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR})

install(TARGETS
  mujoco_system_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(mujoco_ros2_control
  src/mujoco_ros2_control_node.cpp
  src/mujoco_rendering.cpp
  src/mujoco_ros2_control.cpp
)
ament_target_dependencies(mujoco_ros2_control ${THIS_PACKAGE_DEPENDS})
target_link_libraries(mujoco_ros2_control ${MUJOCO_LIB} glfw)
target_include_directories(mujoco_ros2_control
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${MUJOCO_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR})

# Humble compatibility: hardware_interface/version.h is nested in Humble
find_path(HARDWARE_INTERFACE_NESTED_DIR
  NAMES hardware_interface/version.h
  HINTS ${hardware_interface_INCLUDE_DIRS}
  PATH_SUFFIXES hardware_interface
)
if(HARDWARE_INTERFACE_NESTED_DIR)
  target_include_directories(mujoco_system_plugins SYSTEM PUBLIC ${HARDWARE_INTERFACE_NESTED_DIR})
  target_include_directories(mujoco_ros2_control SYSTEM PUBLIC ${HARDWARE_INTERFACE_NESTED_DIR})
endif()

install(TARGETS
  mujoco_ros2_control
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

pluginlib_export_plugin_description_file(mujoco_ros2_control mujoco_system_plugins.xml)

ament_package()
