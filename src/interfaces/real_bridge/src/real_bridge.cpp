// Copyright (c) 2025 SCUTRobotLab
// SPDX-License-Identifier: MIT

#include "real_bridge/real_bridge.hpp"

#include <unistd.h>
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <unordered_map>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "real_bridge/real_uart.h"

namespace template_real_ros2_ctrl {
namespace {

constexpr std::size_t kExpectedJointCount = 8;
constexpr std::uint64_t kMinimumReconnectIntervalMs = 100;
constexpr std::uint64_t kMinimumStateTimeoutMs = 10;

std::uint64_t steady_time_us() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                        std::chrono::steady_clock::now().time_since_epoch())
                                        .count());
}

std::uint64_t wall_time_us() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                                        std::chrono::system_clock::now().time_since_epoch())
                                        .count());
}

double finite_or_zero(double value) { return std::isfinite(value) ? value : 0.0; }

float finite_float_or_zero(double value) {
  if (!std::isfinite(value) || value > std::numeric_limits<float>::max() ||
      value < -std::numeric_limits<float>::max()) {
    return 0.0F;
  }
  return static_cast<float>(value);
}

float safe_gain(double value) { return value >= 0.0 ? finite_float_or_zero(value) : 0.0F; }

int integer_parameter(const std::unordered_map<std::string, std::string>& parameters,
                      const std::string& name, int default_value) {
  const auto iterator = parameters.find(name);
  if (iterator == parameters.end() || iterator->second.empty()) {
    return default_value;
  }
  char* end = nullptr;
  errno = 0;
  const long parsed = std::strtol(iterator->second.c_str(), &end, 10);
  if (errno != 0 || end == iterator->second.c_str() || *end != '\0' ||
      parsed < std::numeric_limits<int>::min() || parsed > std::numeric_limits<int>::max()) {
    RCLCPP_WARN(rclcpp::get_logger("RealBridge"), "Invalid %s='%s'; using %d", name.c_str(),
                iterator->second.c_str(), default_value);
    return default_value;
  }
  return static_cast<int>(parsed);
}

std::string string_parameter(const std::unordered_map<std::string, std::string>& parameters,
                             const std::string& name, const std::string& default_value) {
  const auto iterator = parameters.find(name);
  return iterator == parameters.end() || iterator->second.empty() ? default_value
                                                                  : iterator->second;
}

bool packet_markers_valid(const RealMsgState& packet) {
  return packet.header1 == kPacketHeader1 && packet.header2 == kPacketHeader2 &&
         packet.ender1 == kPacketEnd1 && packet.ender2 == kPacketEnd2;
}

bool packet_payload_finite(const RealMsgState& packet) {
  if (!std::isfinite(packet.h7_timestamp) || !std::isfinite(packet.pc_timestamp)) {
    return false;
  }
  for (const auto& joint : packet.joint_state) {
    if (!std::isfinite(joint.position) || !std::isfinite(joint.velocity) ||
        !std::isfinite(joint.effort)) {
      return false;
    }
  }
  double quaternion_norm_sq = 0.0;
  for (std::size_t index = 0; index < 3; ++index) {
    if (!std::isfinite(packet.imu.linear_acceleration[index]) ||
        !std::isfinite(packet.imu.angular_velocity[index]) ||
        !std::isfinite(packet.imu.orientation[index])) {
      return false;
    }
    quaternion_norm_sq += static_cast<double>(packet.imu.orientation[index]) *
                          static_cast<double>(packet.imu.orientation[index]);
  }
  if (!std::isfinite(packet.imu.orientation[3])) {
    return false;
  }
  quaternion_norm_sq += static_cast<double>(packet.imu.orientation[3]) *
                        static_cast<double>(packet.imu.orientation[3]);
  return quaternion_norm_sq > 1.0e-12 && std::isfinite(packet.dt7_command.cmd_vel_x) &&
         std::isfinite(packet.dt7_command.cmd_omega_z) &&
         std::isfinite(packet.dt7_command.cmd_height);
}

}  // namespace

std::uint16_t crc16(const void* data, std::size_t length, std::uint16_t polynomial) {
  const auto* bytes = static_cast<const std::uint8_t*>(data);
  std::uint16_t crc = 0xFFFF;
  for (std::size_t index = 0; index < length; ++index) {
    crc ^= bytes[index];
    for (std::uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) != 0U ? static_cast<std::uint16_t>((crc >> 1U) ^ polynomial)
                             : static_cast<std::uint16_t>(crc >> 1U);
    }
  }
  return crc;
}

RealBridge::~RealBridge() { close_connection("bridge destroyed"); }

RealBridge::CallbackReturn RealBridge::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (info.joints.size() != kExpectedJointCount) {
    RCLCPP_ERROR(rclcpp::get_logger("RealBridge"), "Expected %zu joints, got %zu",
                 kExpectedJointCount, info.joints.size());
    return CallbackReturn::ERROR;
  }
  if (info.sensors.size() != 2 || info.sensors[0].name != "imu" || info.sensors[1].name != "dt7") {
    RCLCPP_ERROR(rclcpp::get_logger("RealBridge"),
                 "Real backend requires sensors 'imu' and 'dt7' in that order");
    return CallbackReturn::ERROR;
  }

  joints_.resize(info.joints.size());
  for (std::size_t index = 0; index < info.joints.size(); ++index) {
    joints_[index].name = info.joints[index].name;
  }
  imu_name_ = info.sensors[0].name;

  serial_port_ = string_parameter(info.hardware_parameters, "serial_port", "/dev/wheelbipe_h7");
  baudrate_ = integer_parameter(info.hardware_parameters, "baudrate", 2000000);
  const int reconnect_ms =
      std::max(static_cast<int>(kMinimumReconnectIntervalMs),
               integer_parameter(info.hardware_parameters, "serial_reconnect_interval_ms", 1000));
  const int timeout_ms =
      std::max(static_cast<int>(kMinimumStateTimeoutMs),
               integer_parameter(info.hardware_parameters, "state_timeout_ms", 100));
  reconnect_interval_us_ = static_cast<std::uint64_t>(reconnect_ms) * 1000U;
  state_timeout_us_ = static_cast<std::uint64_t>(timeout_ms) * 1000U;

  RCLCPP_INFO(rclcpp::get_logger("RealBridge"),
              "Configured single serial transport: %s at %d baud; DT7 layout is normal-only",
              serial_port_.c_str(), baudrate_);
  return CallbackReturn::SUCCESS;
}

RealBridge::CallbackReturn RealBridge::on_configure(const rclcpp_lifecycle::State&) {
  reset_commands();
  ensure_connected(steady_time_us());
  return CallbackReturn::SUCCESS;
}

RealBridge::CallbackReturn RealBridge::on_cleanup(const rclcpp_lifecycle::State&) {
  close_connection("hardware cleaned up");
  return CallbackReturn::SUCCESS;
}

RealBridge::CallbackReturn RealBridge::on_activate(const rclcpp_lifecycle::State&) {
  reset_commands();
  active_ = true;
  RCLCPP_WARN(rclcpp::get_logger("RealBridge"),
              "Real hardware interface activated; command transmission remains inhibited until "
              "a valid state packet is received");
  return CallbackReturn::SUCCESS;
}

RealBridge::CallbackReturn RealBridge::on_deactivate(const rclcpp_lifecycle::State&) {
  if (serial_fd_ >= 0 && has_valid_state_) {
    transmit(true);
  }
  active_ = false;
  reset_commands();
  close_connection("hardware deactivated");
  return CallbackReturn::SUCCESS;
}

RealBridge::CallbackReturn RealBridge::on_error(const rclcpp_lifecycle::State&) {
  active_ = false;
  reset_commands();
  close_connection("hardware error");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RealBridge::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(joints_.size() * 3U + 14U);
  for (auto& joint : joints_) {
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint.position);
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &joint.velocity);
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &joint.effort);
  }
  interfaces.emplace_back(imu_name_, "orientation.x", &orientation_[0]);
  interfaces.emplace_back(imu_name_, "orientation.y", &orientation_[1]);
  interfaces.emplace_back(imu_name_, "orientation.z", &orientation_[2]);
  interfaces.emplace_back(imu_name_, "orientation.w", &orientation_[3]);
  interfaces.emplace_back(imu_name_, "angular_velocity.x", &angular_velocity_[0]);
  interfaces.emplace_back(imu_name_, "angular_velocity.y", &angular_velocity_[1]);
  interfaces.emplace_back(imu_name_, "angular_velocity.z", &angular_velocity_[2]);
  interfaces.emplace_back(imu_name_, "linear_acceleration.x", &linear_acceleration_[0]);
  interfaces.emplace_back(imu_name_, "linear_acceleration.y", &linear_acceleration_[1]);
  interfaces.emplace_back(imu_name_, "linear_acceleration.z", &linear_acceleration_[2]);
  interfaces.emplace_back("dt7", "cmd_state", &dt7_[0]);
  interfaces.emplace_back("dt7", "cmd_vel_x", &dt7_[1]);
  interfaces.emplace_back("dt7", "cmd_omega_z", &dt7_[2]);
  interfaces.emplace_back("dt7", "cmd_height", &dt7_[3]);
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> RealBridge::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(joints_.size() * 5U);
  for (auto& joint : joints_) {
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION,
                            &joint.position_command);
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY,
                            &joint.velocity_command);
    interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &joint.effort_command);
    interfaces.emplace_back(joint.name, "kp", &joint.kp);
    interfaces.emplace_back(joint.name, "kd", &joint.kd);
  }
  return interfaces;
}

bool RealBridge::ensure_connected(std::uint64_t now_us) {
  if (serial_fd_ >= 0) {
    return true;
  }
  if (last_connect_attempt_us_ != 0 && now_us - last_connect_attempt_us_ < reconnect_interval_us_) {
    return false;
  }
  last_connect_attempt_us_ = now_us;
  serial_fd_ = wheelbipe_serial_open(serial_port_.c_str(), baudrate_);
  if (serial_fd_ < 0) {
    if (!waiting_logged_) {
      RCLCPP_WARN(rclcpp::get_logger("RealBridge"), "Waiting for serial port %s at %d baud: %s",
                  serial_port_.c_str(), baudrate_, std::strerror(errno));
      waiting_logged_ = true;
    }
    return false;
  }
  receive_size_ = 0;
  waiting_logged_ = false;
  RCLCPP_INFO(rclcpp::get_logger("RealBridge"), "Serial port connected: %s", serial_port_.c_str());
  return true;
}

void RealBridge::close_connection(const char* reason) {
  if (serial_fd_ < 0) {
    return;
  }
  wheelbipe_serial_close(serial_fd_);
  serial_fd_ = -1;
  receive_size_ = 0;
  has_valid_state_ = false;
  last_valid_packet_us_ = 0;
  waiting_logged_ = false;
  RCLCPP_WARN(rclcpp::get_logger("RealBridge"), "Serial port closed: %s",
              reason != nullptr ? reason : "unspecified");
}

bool RealBridge::receive_packets() {
  if (serial_fd_ < 0) {
    return false;
  }
  if (receive_size_ == receive_buffer_.size()) {
    receive_size_ = 0;
  }
  const ssize_t received = ::read(serial_fd_, receive_buffer_.data() + receive_size_,
                                  receive_buffer_.size() - receive_size_);
  if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
    close_connection(std::strerror(errno));
    return false;
  }
  if (received > 0) {
    receive_size_ += static_cast<std::size_t>(received);
  }

  bool accepted = false;
  while (receive_size_ >= sizeof(RealMsgState)) {
    std::size_t header = 0;
    while (header + 1U < receive_size_ && (receive_buffer_[header] != kPacketHeader1 ||
                                           receive_buffer_[header + 1U] != kPacketHeader2)) {
      ++header;
    }
    if (header > 0) {
      std::memmove(receive_buffer_.data(), receive_buffer_.data() + header, receive_size_ - header);
      receive_size_ -= header;
    }
    if (receive_size_ < sizeof(RealMsgState)) {
      break;
    }

    RealMsgState candidate{};
    std::memcpy(&candidate, receive_buffer_.data(), sizeof(candidate));
    const bool valid = packet_markers_valid(candidate) && packet_payload_finite(candidate) &&
                       crc16(&candidate, sizeof(candidate) - 4U) == candidate.crc16;
    const std::size_t consumed = valid ? sizeof(candidate) : 1U;
    std::memmove(receive_buffer_.data(), receive_buffer_.data() + consumed,
                 receive_size_ - consumed);
    receive_size_ -= consumed;
    if (valid) {
      latest_state_ = candidate;
      latest_h7_timestamp_ = candidate.h7_timestamp;
      last_valid_packet_us_ = steady_time_us();
      has_valid_state_ = true;
      accepted = true;
    }
  }
  return accepted;
}

void RealBridge::apply_latest_state() {
  for (std::size_t index = 0; index < kCommunicatedJointCount; ++index) {
    joints_[index].position =
        finite_or_zero(std::remainder(latest_state_.joint_state[index].position, 2.0 * M_PI));
    joints_[index].velocity = finite_or_zero(latest_state_.joint_state[index].velocity);
    joints_[index].effort = finite_or_zero(latest_state_.joint_state[index].effort);
  }
  for (std::size_t index = kCommunicatedJointCount; index < joints_.size(); ++index) {
    joints_[index].position = 0.0;
    joints_[index].velocity = 0.0;
    joints_[index].effort = 0.0;
  }

  for (std::size_t index = 0; index < 3; ++index) {
    linear_acceleration_[index] = finite_or_zero(latest_state_.imu.linear_acceleration[index]);
    angular_velocity_[index] = finite_or_zero(latest_state_.imu.angular_velocity[index]);
  }
  orientation_[0] = finite_or_zero(-latest_state_.imu.orientation[1]);
  orientation_[1] = finite_or_zero(latest_state_.imu.orientation[0]);
  orientation_[2] = finite_or_zero(latest_state_.imu.orientation[2]);
  orientation_[3] = finite_or_zero(latest_state_.imu.orientation[3]);

  dt7_[0] = static_cast<double>(latest_state_.dt7_command.cmd_state);
  dt7_[1] = finite_or_zero(latest_state_.dt7_command.cmd_vel_x);
  dt7_[2] = finite_or_zero(latest_state_.dt7_command.cmd_omega_z);
  dt7_[3] = finite_or_zero(latest_state_.dt7_command.cmd_height);
}

hardware_interface::return_type RealBridge::read(const rclcpp::Time&, const rclcpp::Duration&) {
  if (!ensure_connected(steady_time_us())) {
    return hardware_interface::return_type::OK;
  }
  if (receive_packets()) {
    apply_latest_state();
  }
  const std::uint64_t now_us = steady_time_us();
  if (has_valid_state_ && now_us - last_valid_packet_us_ > state_timeout_us_) {
    has_valid_state_ = false;
    RCLCPP_ERROR(rclcpp::get_logger("RealBridge"),
                 "H7 state packet timed out; command output is inhibited");
  }
  return hardware_interface::return_type::OK;
}

bool RealBridge::transmit(bool safe_stop) {
  if (serial_fd_ < 0) {
    return false;
  }
  RealMsgCommand packet{};
  packet.header1 = kPacketHeader1;
  packet.header2 = kPacketHeader2;
  packet.h7_timestamp = latest_h7_timestamp_;
  packet.pc_timestamp = static_cast<double>(wall_time_us());
  if (!safe_stop) {
    for (std::size_t index = 0; index < kCommunicatedJointCount; ++index) {
      packet.joint_command[index].position_command =
          finite_float_or_zero(joints_[index].position_command);
      packet.joint_command[index].velocity_command =
          finite_float_or_zero(joints_[index].velocity_command);
      packet.joint_command[index].effort_command =
          finite_float_or_zero(joints_[index].effort_command);
      packet.joint_command[index].kp = safe_gain(joints_[index].kp);
      packet.joint_command[index].kd = safe_gain(joints_[index].kd);
    }
  }
  packet.ender1 = kPacketEnd1;
  packet.ender2 = kPacketEnd2;
  packet.crc16 = crc16(&packet, sizeof(packet) - 4U);
  const int result = wheelbipe_serial_write(serial_fd_, &packet, sizeof(packet));
  if (result != static_cast<int>(sizeof(packet))) {
    close_connection(std::strerror(errno));
    return false;
  }
  return true;
}

hardware_interface::return_type RealBridge::write(const rclcpp::Time&, const rclcpp::Duration&) {
  if (!active_ || !ensure_connected(steady_time_us())) {
    return hardware_interface::return_type::OK;
  }
  if (!has_valid_state_) {
    return hardware_interface::return_type::OK;
  }
  transmit(false);
  return hardware_interface::return_type::OK;
}

void RealBridge::reset_commands() {
  for (auto& joint : joints_) {
    joint.position_command = 0.0;
    joint.velocity_command = 0.0;
    joint.effort_command = 0.0;
    joint.kp = 0.0;
    joint.kd = 0.0;
  }
}

}  // namespace template_real_ros2_ctrl

PLUGINLIB_EXPORT_CLASS(template_real_ros2_ctrl::RealBridge, hardware_interface::SystemInterface)
