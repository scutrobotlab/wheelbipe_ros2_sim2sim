// Copyright (c) 2025 SCUTRobotLab
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "fsm/state_base.hpp"

#include "fsm/state_machine.hpp"

namespace robot_locomotion {

StateBase::StateBase(StateMachine* state_machine, rclcpp::Logger logger)
    : state_machine_(state_machine), logger_(logger) {}

}  // namespace robot_locomotion
