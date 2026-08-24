// Copyright (c) 2025 Sangtaek Lee
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_HPP_

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "controller_manager/controller_manager.hpp"
#include "mujoco/mujoco.h"
#include "mujoco_ros2_control/mujoco_system.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace mujoco_ros2_control {
class MujocoRos2Control {
 public:
  MujocoRos2Control(rclcpp::Node::SharedPtr& node, mjModel* mujoco_model, mjData* mujoco_data);
  ~MujocoRos2Control();
  bool init();
  void update();
  void reset_sim_time();

 private:
  void publish_sim_time(rclcpp::Time sim_time);
  void publish_imu_if_enabled(const rclcpp::Time& sim_time);
  void publish_base_state_if_enabled();
  void publish_wheel_contact_force_if_enabled();
  double wheel_ground_contact_force_magnitude(int wheel_geom_id) const;
  std::string get_robot_description();
  rclcpp::Node::SharedPtr node_;
  mjModel* mj_model_;
  mjData* mj_data_;

  rclcpp::Logger logger_;
  std::shared_ptr<pluginlib::ClassLoader<MujocoSystemInterface>> robot_hw_sim_loader_;

  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  rclcpp::Executor::SharedPtr cm_executor_;
  std::thread cm_thread_;
  std::atomic_bool stop_cm_thread_{false};
  bool initialized_{false};
  rclcpp::Duration control_period_;

  rclcpp::Time last_update_sim_time_ros_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;

  /// Optional direct IMU topic (same MJCF sensors as ros2_control: framequat "imu", gyro,
  /// accelerometer)
  bool publish_imu_{false};
  int imu_quat_sensor_adr_{-1};
  int gyro_sensor_adr_{-1};
  int accel_sensor_adr_{-1};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  std::string imu_frame_id_;

  bool publish_base_state_{true};
  int base_body_id_{-1};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr base_state_publisher_;

  bool publish_wheel_contact_force_{true};
  std::array<int, 2> wheel_geom_ids_{{-1, -1}};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_contact_force_publisher_;
};
}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_ROS2_CONTROL_HPP_
