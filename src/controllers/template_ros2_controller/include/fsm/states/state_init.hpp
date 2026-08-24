// Copyright (c) 2025 SCUTRobotLab
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef FSM__STATES__STATE_INIT_HPP_
#define FSM__STATES__STATE_INIT_HPP_

#include <string>

#include "fsm/state_base.hpp"

namespace robot_locomotion {

// 初始化状态类
class StateInit : public StateBase {
 public:
  StateInit(StateMachine* state_machine, rclcpp::Logger logger);
  virtual ~StateInit() = default;

  void enter(const RobotState& robot_state, const rclcpp::Time& time) override;
  void run(RobotState& robot_state, const rclcpp::Time& time,
           const rclcpp::Duration& period) override;
  void exit(const RobotState& robot_state, const rclcpp::Time& time) override;
  std::string getName() const override { return "INIT"; }
};

}  // namespace robot_locomotion

#endif  // FSM__STATES__STATE_INIT_HPP_
