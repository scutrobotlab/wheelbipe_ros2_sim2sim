// Copyright (c) 2025 SCUTRobotLab
// SPDX-License-Identifier: MIT

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "real_bridge/real_msg.hpp"

namespace template_real_ros2_ctrl {

struct JointData {
  std::string name;
  double position = 0.0;
  double velocity = 0.0;
  double effort = 0.0;
  double position_command = 0.0;
  double velocity_command = 0.0;
  double effort_command = 0.0;
  double kp = 0.0;
  double kd = 0.0;
};

class RealBridge final : public hardware_interface::SystemInterface {
 public:
  RealBridge() = default;
  ~RealBridge() override;

  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

 private:
  bool ensure_connected(std::uint64_t now_us);
  void close_connection(const char* reason);
  bool receive_packets();
  bool transmit(bool safe_stop);
  void apply_latest_state();
  void reset_commands();

  std::vector<JointData> joints_;
  std::string imu_name_;
  std::array<double, 3> linear_acceleration_{};
  std::array<double, 3> angular_velocity_{};
  std::array<double, 4> orientation_{0.0, 0.0, 0.0, 1.0};
  std::array<double, 4> dt7_{};

  std::string serial_port_ = "/dev/wheelbipe_h7";
  int baudrate_ = 2000000;
  int serial_fd_ = -1;
  std::uint64_t reconnect_interval_us_ = 1000000;
  std::uint64_t state_timeout_us_ = 100000;
  std::uint64_t last_connect_attempt_us_ = 0;
  std::uint64_t last_valid_packet_us_ = 0;
  bool waiting_logged_ = false;
  bool active_ = false;
  bool has_valid_state_ = false;

  std::array<std::uint8_t, 1024> receive_buffer_{};
  std::size_t receive_size_ = 0;
  RealMsgState latest_state_{};
  float latest_h7_timestamp_ = 0.0F;
};

}  // namespace template_real_ros2_ctrl
