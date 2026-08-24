// Copyright (c) 2025 SCUTRobotLab
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "fsm/states/state_init.hpp"

#include "robot_state/robot_state.hpp"

namespace robot_locomotion {

StateInit::StateInit(StateMachine* state_machine, rclcpp::Logger logger)
    : StateBase(state_machine, logger) {}

void StateInit::enter(const RobotState& robot_state, const rclcpp::Time& time) {
  (void)robot_state;
  (void)time;
  RCLCPP_INFO(logger_, "Entering INIT state");
}

void StateInit::run(RobotState& robot_state, const rclcpp::Time& time,
                    const rclcpp::Duration& period) {
  (void)time;
  (void)period;
  // 初始化状态：所有力矩设为0
  for (auto& joint : robot_state.joints) {
    joint.output_torque = 0.0;
  }
}

void StateInit::exit(const RobotState& robot_state, const rclcpp::Time& time) {
  (void)robot_state;
  (void)time;
  RCLCPP_INFO(logger_, "Exiting INIT state");
}

}  // namespace robot_locomotion
