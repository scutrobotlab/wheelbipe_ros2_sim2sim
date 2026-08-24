// Copyright (c) 2025 SCUTRobotLab
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "fsm/states/state_idle.hpp"

#include "robot_state/robot_state.hpp"

namespace robot_locomotion {

StateIdle::StateIdle(StateMachine* state_machine, rclcpp::Logger logger)
    : StateBase(state_machine, logger) {}

void StateIdle::enter(const RobotState& robot_state, const rclcpp::Time& time) {
  (void)robot_state;
  (void)time;
  RCLCPP_INFO(logger_, "Entering IDLE state");
}

void StateIdle::run(RobotState& robot_state, const rclcpp::Time& time,
                    const rclcpp::Duration& period) {
  (void)time;
  (void)period;
  // 空闲状态：零力矩/零增益输出。位置命令仅随当前测量刷新，不产生 PD 保持力。
  for (auto& joint : robot_state.joints) {
    joint.output_position = joint.position;
    joint.output_velocity = 0.0;
    joint.output_torque = 0.0;
    joint.output_kp = 0.0;
    joint.output_kd = 0.0;
  }
}

void StateIdle::exit(const RobotState& robot_state, const rclcpp::Time& time) {
  (void)robot_state;
  (void)time;
  RCLCPP_INFO(logger_, "Exiting IDLE state");
}

}  // namespace robot_locomotion
