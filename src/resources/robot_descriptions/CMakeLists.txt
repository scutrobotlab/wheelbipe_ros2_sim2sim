cmake_minimum_required(VERSION 3.8)
project(robot_descriptions)

find_package(ament_cmake REQUIRED)

set(WHEELBIPE_MESH_FILES
  wheelbipeV14_2/meshes/ASSET_LICENSE.md
  wheelbipeV14_2/meshes/base_link.STL
  wheelbipeV14_2/meshes/gimbal_pitch_link.STL
  wheelbipeV14_2/meshes/gimbal_yaw_link.STL
  wheelbipeV14_2/meshes/left_front1_link.STL
  wheelbipeV14_2/meshes/left_front2_link.STL
  wheelbipeV14_2/meshes/left_front3_link.STL
  wheelbipeV14_2/meshes/left_front4_link.STL
  wheelbipeV14_2/meshes/left_rear1_link.STL
  wheelbipeV14_2/meshes/left_rear2_link.STL
  wheelbipeV14_2/meshes/left_spring1_link.STL
  wheelbipeV14_2/meshes/left_spring2_link.STL
  wheelbipeV14_2/meshes/left_wheel_link.STL
  wheelbipeV14_2/meshes/right_front1_link.STL
  wheelbipeV14_2/meshes/right_front2_link.STL
  wheelbipeV14_2/meshes/right_front3_link.STL
  wheelbipeV14_2/meshes/right_front4_link.STL
  wheelbipeV14_2/meshes/right_rear1_link.STL
  wheelbipeV14_2/meshes/right_rear2_link.STL
  wheelbipeV14_2/meshes/right_spring1_link.STL
  wheelbipeV14_2/meshes/right_spring2_link.STL
  wheelbipeV14_2/meshes/right_wheel_link.STL
)
install(
  FILES ${WHEELBIPE_MESH_FILES}
  DESTINATION share/${PROJECT_NAME}/wheelbipeV14_2/meshes
)

install(
  FILES
    wheelbipeV14_2/mjcf/scene.xml
    wheelbipeV14_2/mjcf/wheelbipeV14_2.xml
  DESTINATION share/${PROJECT_NAME}/wheelbipeV14_2/mjcf
)

install(
  FILES
    wheelbipe_V14/xacro/description.xacro
    wheelbipe_V14/xacro/robot_real.xacro
    wheelbipe_V14/xacro/robot_sim.xacro
    wheelbipe_V14/xacro/ros2control.xacro
  DESTINATION share/${PROJECT_NAME}/wheelbipe_V14/xacro
)

ament_package()
