// Copyright (c) 2025 SCUTRobotLab
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef FSM__STATE_BASE_HPP_
#define FSM__STATE_BASE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace robot_locomotion {

// 前向声明
struct RobotState;
class StateMachine;

// 状态基类
class StateBase {
 public:
  StateBase(StateMachine* state_machine, rclcpp::Logger logger);
  virtual ~StateBase() = default;

  // 进入状态时调用一次
  virtual void enter(const RobotState& robot_state, const rclcpp::Time& time) = 0;

  // 状态运行时循环调用
  virtual void run(RobotState& robot_state, const rclcpp::Time& time,
                   const rclcpp::Duration& period) = 0;

  // 退出状态时调用一次
  virtual void exit(const RobotState& robot_state, const rclcpp::Time& time) = 0;

  // 获取状态名称
  virtual std::string getName() const = 0;

 protected:
  StateMachine* state_machine_;
  rclcpp::Logger logger_;
};

}  // namespace robot_locomotion

#endif  // FSM__STATE_BASE_HPP_
