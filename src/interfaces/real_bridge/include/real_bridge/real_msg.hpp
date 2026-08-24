// Copyright (c) 2025 SCUTRobotLab
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace template_real_ros2_ctrl {

constexpr std::uint8_t kPacketHeader1 = 0xA8;
constexpr std::uint8_t kPacketHeader2 = 0xE6;
constexpr std::uint8_t kPacketEnd1 = 0xC3;
constexpr std::uint8_t kPacketEnd2 = 0xF7;
constexpr std::size_t kCommunicatedJointCount = 6;

#pragma pack(push, 1)

struct RealJointState {
  float position;
  float velocity;
  float effort;
};

struct RealJointCommand {
  float position_command;
  float velocity_command;
  float effort_command;
  float kp;
  float kd;
};

// Ordinary DT7 command. The public deployment protocol intentionally has no jump field.
struct RealDt7Command {
  std::uint8_t cmd_state;
  float cmd_vel_x;
  float cmd_omega_z;
  float cmd_height;
};

struct RealImu {
  float linear_acceleration[3];
  float angular_velocity[3];
  float orientation[4];  // x, y, z, w
};

// Keep the latest H7 wire layout. Optional tuning values are transmitted as zero by this
// simplified bridge so the packet stays compatible without exposing extra ROS interfaces.
struct RealMsgCommand {
  std::uint8_t header1;
  std::uint8_t header2;
  float h7_timestamp;
  double pc_timestamp;
  RealJointCommand joint_command[kCommunicatedJointCount];
  float spring_compensation_scale[2];
  float speed_error_acc;
  float speed_error_brake;
  float speed_error_turn;
  std::uint16_t crc16;
  std::uint8_t ender1;
  std::uint8_t ender2;
};

struct RealMsgState {
  std::uint8_t header1;
  std::uint8_t header2;
  float h7_timestamp;
  double pc_timestamp;
  RealJointState joint_state[kCommunicatedJointCount];
  RealImu imu;
  RealDt7Command dt7_command;
  std::uint16_t crc16;
  std::uint8_t ender1;
  std::uint8_t ender2;
};

#pragma pack(pop)

static_assert(std::is_trivially_copyable_v<RealMsgCommand>);
static_assert(std::is_trivially_copyable_v<RealMsgState>);
static_assert(sizeof(RealJointState) == 12);
static_assert(sizeof(RealJointCommand) == 20);
static_assert(sizeof(RealDt7Command) == 13);
static_assert(sizeof(RealImu) == 40);
static_assert(sizeof(RealMsgCommand) == 158);
static_assert(sizeof(RealMsgState) == 143);

std::uint16_t crc16(const void* data, std::size_t length, std::uint16_t polynomial = 0x8005);

}  // namespace template_real_ros2_ctrl
