// Copyright (c) 2025 SCUTRobotLab
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef FSM__STATES__STATE_IDLE_HPP_
#define FSM__STATES__STATE_IDLE_HPP_

#include <string>

#include "fsm/state_base.hpp"

namespace robot_locomotion {

// 空闲状态类
class StateIdle : public StateBase {
 public:
  StateIdle(StateMachine* state_machine, rclcpp::Logger logger);
  virtual ~StateIdle() = default;

  void enter(const RobotState& robot_state, const rclcpp::Time& time) override;
  void run(RobotState& robot_state, const rclcpp::Time& time,
           const rclcpp::Duration& period) override;
  void exit(const RobotState& robot_state, const rclcpp::Time& time) override;
  std::string getName() const override { return "IDLE"; }
};

}  // namespace robot_locomotion

#endif  // FSM__STATES__STATE_IDLE_HPP_
