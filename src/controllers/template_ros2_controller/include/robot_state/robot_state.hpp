// Copyright (c) 2025 SCUTRobotLab
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROBOT_STATE__ROBOT_STATE_HPP_
#define ROBOT_STATE__ROBOT_STATE_HPP_

#include <array>
#include <string>
#include <vector>

#include "utils/cppTypes.h"

namespace robot_locomotion {

// 关节状态结构
struct JointState {
  std::string name;              // 关节名称
  double position = 0.0;         // 当前关节位置 (rad)
  double velocity = 0.0;         // 当前关节速度 (rad/s)
  double effort = 0.0;           // 当前关节力矩 (N·m)
  double state_timestamp = 0.0;  // 硬件/仿真状态时间戳（优先来自 timeStamp 接口）
  double output_torque = 0.0;    // 控制器输出力矩 (N·m)
  double output_position = 0.0;  // 控制器期望位置（目标角度，rad）
  double output_velocity = 0.0;  // 控制器期望速度（目标角速度，rad/s）
  double output_kp = 0.0;        // 控制器下发 Kp
  double output_kd = 0.0;        // 控制器下发 Kd
  double command_timestamp = 0.0;  // 命令写入时间戳（通常为 PC 微秒时间）
};

// IMU传感器状态结构
struct IMUState {
  std::array<double, 3> linear_acceleration = {0.0, 0.0, 0.0};  // 线性加速度 (m/s²)
  std::array<double, 3> angular_velocity = {0.0, 0.0, 0.0};     // 角速度 (rad/s)
  std::array<double, 4> orientation = {0.0, 0.0, 0.0, 1.0};     // 四元数 (x, y, z, w)
};

// 全身广义状态
struct GeneralizedState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec3<double> ang_vel_b;
  Vec3<double> lin_acc_b;
  Quat<double> orientation_b;
  Vec3<double> rpy;
  RotMat<double> rotation_b2w;  // 机身系转到世界系的旋转矩阵
  RotMat<double> rotation_w2b;  // 世界系转到机身系的旋转矩阵
};

// 运动指令：机体速度和目标高度。
struct Command {
  std::array<double, 3> cmd_vel = {0.0, 0.0, 0.0};  // 速度指令
  double cmd_height = 0.0;                          // 高度指令
};

// 机器人状态数据结构（暴露给状态机）
struct RobotState {
  // 关节状态
  std::vector<JointState> joints;

  // 传感器状态
  IMUState imu;

  // 广义状态
  GeneralizedState body_state;

  // 运动指令
  Command command;

  // 时间信息
  double timestamp = 0.0;  // 时间戳 (s)
  double period = 0.0;     // 控制周期 (s)

  // 计算函数：计算各种状态表示
  void run();
};

}  // namespace robot_locomotion

#endif  // ROBOT_STATE__ROBOT_STATE_HPP_
