cmake_minimum_required(VERSION 3.8)
project(template_ros2_controller)

option(WHEELBIPE_INSTALL_POLICIES "Install bundled policy files" ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

generate_parameter_library(template_ros2_controller_parameters
  config/template_ros2_controller_parameters.yaml
)

pluginlib_export_plugin_description_file(
  controller_interface template_ros2_controller_plugin.xml
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/fsm/state_base.cpp
  src/fsm/state_machine.cpp
  src/fsm/states/state_idle.cpp
  src/fsm/states/state_init.cpp
  src/fsm/states/state_prepare.cpp
  src/fsm/states/state_rl.cpp
  src/onnxruntime/onnxruntime_inference.cpp
  src/robot_state/robot_state.cpp
  src/template_ros2_controller.cpp
  src/utils/noise_generator.cpp
)

# In ROS 2 Humble the version header is installed one directory deeper than
# in newer hardware_interface releases.
find_path(HARDWARE_INTERFACE_NESTED_DIR
  NAMES hardware_interface/version.h
  HINTS ${hardware_interface_INCLUDE_DIRS}
  PATH_SUFFIXES hardware_interface
)
if(HARDWARE_INTERFACE_NESTED_DIR)
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${HARDWARE_INTERFACE_NESTED_DIR}
  )
endif()

set(ONNXRUNTIME_EXPLICIT_ROOT "")
if(ONNXRUNTIME_ROOT)
  set(ONNXRUNTIME_EXPLICIT_ROOT "${ONNXRUNTIME_ROOT}")
elseif(DEFINED ENV{ONNXRUNTIME_ROOT} AND NOT "$ENV{ONNXRUNTIME_ROOT}" STREQUAL "")
  set(ONNXRUNTIME_EXPLICIT_ROOT "$ENV{ONNXRUNTIME_ROOT}")
endif()

if(ONNXRUNTIME_EXPLICIT_ROOT)
  find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
    PATHS "${ONNXRUNTIME_EXPLICIT_ROOT}/include"
    NO_DEFAULT_PATH
  )
  find_library(ONNXRUNTIME_LIBRARY onnxruntime
    PATHS "${ONNXRUNTIME_EXPLICIT_ROOT}/lib"
    NO_DEFAULT_PATH
  )

  set(ONNXRUNTIME_VERSION_FILE "${ONNXRUNTIME_EXPLICIT_ROOT}/VERSION_NUMBER")
  if(EXISTS "${ONNXRUNTIME_VERSION_FILE}")
    file(STRINGS "${ONNXRUNTIME_VERSION_FILE}" ONNXRUNTIME_VERSION LIMIT_COUNT 1)
    if(NOT ONNXRUNTIME_VERSION STREQUAL "1.20.0")
      message(FATAL_ERROR
        "WheelBipe requires ONNX Runtime 1.20.0; found ${ONNXRUNTIME_VERSION}"
      )
    endif()
  endif()
else()
  find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
    PATHS
      /usr/include/onnxruntime
      /usr/local/include/onnxruntime
      /opt/ros/humble/opt/onnxruntime_vendor/include
  )
  find_library(ONNXRUNTIME_LIBRARY onnxruntime
    PATHS
      /usr/lib
      /usr/local/lib
      /opt/ros/humble/opt/onnxruntime_vendor/lib
  )
endif()

if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
  message(FATAL_ERROR
    "ONNX Runtime was not found. Set ONNXRUNTIME_ROOT to the 1.20.0 package root."
  )
endif()

target_include_directories(${PROJECT_NAME} PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}
  template_ros2_controller_parameters
  ${ONNXRUNTIME_LIBRARY}
)
get_filename_component(ONNXRUNTIME_LIBRARY_DIR ${ONNXRUNTIME_LIBRARY} DIRECTORY)
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_LIBRARY_DIR}"
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
if(WHEELBIPE_INSTALL_POLICIES)
  install(
    FILES policy/ASSET_LICENSE.md
    DESTINATION share/${PROJECT_NAME}/policy
  )
  install(
    FILES policy/parallel/V14-35-flat-and-rotation-13k.onnx
    DESTINATION share/${PROJECT_NAME}/policy/parallel
  )
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
