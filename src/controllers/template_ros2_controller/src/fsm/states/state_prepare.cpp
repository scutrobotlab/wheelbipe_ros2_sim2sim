// Copyright (c) 2025 SCUTRobotLab
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "fsm/states/state_prepare.hpp"

#include <algorithm>
#include <cmath>

#include "fsm/state_machine.hpp"
#include "robot_state/robot_state.hpp"

namespace robot_locomotion {

StatePrepare::StatePrepare(StateMachine* state_machine, rclcpp::Logger logger)
    : StateBase(state_machine, logger) {
  desired_pos_.resize(NUM_PREPARE_JOINTS, 0.0);
}

void StatePrepare::enter(const RobotState& robot_state, const rclcpp::Time& time) {
  (void)time;
  // 读取前四关节当前位置作为插值起点
  const size_t n = std::min(NUM_PREPARE_JOINTS, robot_state.joints.size());
  desired_pos_.resize(NUM_PREPARE_JOINTS, 0.0);
  for (size_t i = 0; i < n; ++i) {
    desired_pos_[i] = robot_state.joints[i].position;
  }
  RCLCPP_INFO(logger_, "Entering PREPARE state: first 4 joint positions recorded");
}

void StatePrepare::run(RobotState& robot_state, const rclcpp::Time& time,
                       const rclcpp::Duration& period) {
  (void)time;
  const double dt = period.seconds();
  const std::vector<double>& target = state_machine_->getPrepareTargetPos();
  const std::vector<double>& kp = state_machine_->getPrepareKp();
  const std::vector<double>& kd = state_machine_->getPrepareKd();
  const double max_vel = state_machine_->getPrepareMaxVelocity();

  const size_t n = std::min({NUM_PREPARE_JOINTS, robot_state.joints.size(), target.size(),
                             kp.size(), kd.size(), desired_pos_.size()});

  for (size_t i = 0; i < n; ++i) {
    double err = target[i] - desired_pos_[i];
    double step = std::copysign(std::min(std::abs(err), max_vel * dt), err);
    desired_pos_[i] += step;
  }

  for (size_t i = 0; i < robot_state.joints.size(); ++i) {
    if (i < n) {
      double pos_err = desired_pos_[i] - robot_state.joints[i].position;
      double vel = -robot_state.joints[i].velocity;
      robot_state.joints[i].output_torque = kp[i] * pos_err + kd[i] * vel;
      robot_state.joints[i].output_position = desired_pos_[i];
    } else if (i == 6 || i == 7) {
      const auto& joint_bias = state_machine_->getJointBias();
      robot_state.joints[i].output_torque = (i < joint_bias.size()) ? joint_bias[i] : 0.0;
    } else {
      robot_state.joints[i].output_torque = 0.0;
    }
  }
}

void StatePrepare::exit(const RobotState& robot_state, const rclcpp::Time& time) {
  (void)robot_state;
  (void)time;
  RCLCPP_INFO(logger_, "Exiting PREPARE state");
}

}  // namespace robot_locomotion
