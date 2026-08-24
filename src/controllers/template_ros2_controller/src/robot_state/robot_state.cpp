// Copyright (c) 2025 SCUTRobotLab
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "robot_state/robot_state.hpp"

#include "utils/orientation_tools.h"

namespace robot_locomotion {

void RobotState::run() {
  this->body_state.ang_vel_b = Eigen::Map<const Vec3<double>>(imu.angular_velocity.data());
  this->body_state.lin_acc_b = Eigen::Map<const Vec3<double>>(imu.linear_acceleration.data());
  this->body_state.orientation_b = Eigen::Map<const Quat<double>>(imu.orientation.data());
  // 计算旋转矩阵
  this->body_state.rotation_w2b = ori::quaternionToRotationMatrix(this->body_state.orientation_b);
  this->body_state.rotation_b2w = this->body_state.rotation_w2b.transpose();
  // 计算滚转俯仰偏航角
  this->body_state.rpy = ori::quatToRPY(this->body_state.orientation_b);
}

}  // namespace robot_locomotion
