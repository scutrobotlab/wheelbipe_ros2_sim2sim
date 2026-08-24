// Copyright (c) 2025 SCUTRobotLab
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef TEMPLATE_ROS2_CONTROLLER__TEMPLATE_ROS2_CONTROLLER_HPP_
#define TEMPLATE_ROS2_CONTROLLER__TEMPLATE_ROS2_CONTROLLER_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <semantic_components/imu_sensor.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <template_ros2_controller/template_ros2_controller_parameters.hpp>

#include "fsm/state_machine.hpp"
#include "robot_state/robot_state.hpp"

namespace robot_locomotion {

using LoanedCommandInterface =
    std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>;
using LoanedStateInterface =
    std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>>;

struct Joint {
  std::string name;
  LoanedCommandInterface position_command_handle;
  LoanedCommandInterface velocity_command_handle;
  LoanedCommandInterface effort_command_handle;
  LoanedCommandInterface kp_command_handle;
  LoanedCommandInterface kd_command_handle;
  LoanedStateInterface position_handle;
  LoanedStateInterface velocity_handle;
  LoanedStateInterface effort_handle;
};

struct JointCommandSnapshot {
  double position = 0.0;
  double velocity = 0.0;
  double effort = 0.0;
  double kp = 0.0;
  double kd = 0.0;
};

[[nodiscard]] bool validateControllerParameters(const template_ros2_controller::Params& params,
                                                std::string& error);
[[nodiscard]] bool validateJointContract(const std::vector<std::string>& joint_names,
                                         std::string& error);
[[nodiscard]] geometry_msgs::msg::Twist limitMotionCommand(
    const geometry_msgs::msg::Twist& command, const template_ros2_controller::Params& params);
[[nodiscard]] double limitHeightCommand(double height,
                                        const template_ros2_controller::Params& params);

class TemplateRos2Controller : public controller_interface::ControllerInterface {
 public:
  TemplateRos2Controller() = default;
  ~TemplateRos2Controller() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State& previous_state) override;

 private:
  void configureStateMachine();
  bool updateRobotState(const rclcpp::Time& time, const rclcpp::Duration& period);
  void writeControllerCommands(ControllerState state);
  void writeSafeJointCommands();
  void resetControllerRuntime(const rclcpp::Time& time);
  void motionCommandCallback(const geometry_msgs::msg::Twist::SharedPtr message);
  void heightCommandCallback(const std_msgs::msg::Float64::SharedPtr message);
  void stateCommandCallback(const std_msgs::msg::Int32::SharedPtr message);
  bool resolveStateCommand(int raw_state, ControllerState& target) const;

  std::vector<std::shared_ptr<Joint>> joints_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> sensor_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;
  std::unique_ptr<semantic_components::IMUSensor> imu_sensor_;
  LoanedStateInterface dt7_cmd_state_handle_;
  LoanedStateInterface dt7_cmd_vel_x_handle_;
  LoanedStateInterface dt7_cmd_omega_z_handle_;
  LoanedStateInterface dt7_cmd_height_handle_;
  int dt7_previous_state_ = -1;

  std::shared_ptr<template_ros2_controller::ParamListener> param_listener_;
  template_ros2_controller::Params params_;
  std::string controller_param_prefix_;

  std::unique_ptr<StateMachine> state_machine_;
  RobotState robot_state_;
  std::vector<JointCommandSnapshot> joint_command_snapshots_;
  rclcpp::Time last_update_time_;
  bool rl_inference_ready_ = false;
  bool auto_enter_rl_pending_ = false;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr height_cmd_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr state_cmd_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr current_state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_final_torque_publisher_;

  std::mutex motion_cmd_mutex_;
  double latest_lin_vel_x_ = 0.0;
  double latest_ang_vel_z_ = 0.0;
  double latest_height_ = 0.22;
  double motion_command_timeout_sec_ = 0.5;
  std::uint64_t last_motion_command_receive_ns_ = 0;
  std::uint64_t last_height_command_receive_ns_ = 0;
  bool motion_cmd_received_ = false;
  bool height_cmd_received_ = false;
};

}  // namespace robot_locomotion

#endif  // TEMPLATE_ROS2_CONTROLLER__TEMPLATE_ROS2_CONTROLLER_HPP_
