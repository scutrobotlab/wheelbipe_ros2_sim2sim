// Copyright (c) 2025 SCUTRobotLab
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef FSM__STATES__STATE_PREPARE_HPP_
#define FSM__STATES__STATE_PREPARE_HPP_

#include <string>
#include <vector>

#include "fsm/state_base.hpp"

namespace robot_locomotion {

// 预备状态：读取前四关节当前位置，以指定速度 PD 复位到可配置初始值
class StatePrepare : public StateBase {
 public:
  static constexpr size_t NUM_PREPARE_JOINTS = 4;

  StatePrepare(StateMachine* state_machine, rclcpp::Logger logger);
  virtual ~StatePrepare() = default;

  void enter(const RobotState& robot_state, const rclcpp::Time& time) override;
  void run(RobotState& robot_state, const rclcpp::Time& time,
           const rclcpp::Duration& period) override;
  void exit(const RobotState& robot_state, const rclcpp::Time& time) override;
  std::string getName() const override { return "PREPARE"; }

 private:
  // 当前期望位置（向目标插值），仅前四关节
  std::vector<double> desired_pos_;
};

}  // namespace robot_locomotion

#endif  // FSM__STATES__STATE_PREPARE_HPP_
