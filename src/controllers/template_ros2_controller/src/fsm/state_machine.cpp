// Copyright (c) 2025 SCUTRobotLab
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "fsm/state_machine.hpp"

#include "fsm/state_base.hpp"
#include "fsm/states/state_idle.hpp"
#include "fsm/states/state_init.hpp"
#include "fsm/states/state_prepare.hpp"
#include "fsm/states/state_rl.hpp"
#include "onnxruntime/onnxruntime_inference.hpp"
#include "robot_state/robot_state.hpp"

namespace robot_locomotion {
namespace {

StateRL* rlState(std::map<ControllerState, std::unique_ptr<StateBase>>& states) {
  const auto it = states.find(ControllerState::RL);
  return it == states.end() ? nullptr : dynamic_cast<StateRL*>(it->second.get());
}

}  // namespace

StateMachine::StateMachine(rclcpp::Logger logger, rclcpp_lifecycle::LifecycleNode::SharedPtr node)
    : logger_(logger), node_(node), state_entry_time_(0, 0, RCL_ROS_TIME) {
  initializeStates();
  current_state_obj_ = states_.at(ControllerState::INIT).get();
}

StateMachine::~StateMachine() { stopRLInference(); }

void StateMachine::initializeStates() {
  states_[ControllerState::INIT] = std::make_unique<StateInit>(this, logger_);
  states_[ControllerState::IDLE] = std::make_unique<StateIdle>(this, logger_);
  states_[ControllerState::PREPARE] = std::make_unique<StatePrepare>(this, logger_);
  states_[ControllerState::RL] = std::make_unique<StateRL>(this, logger_, node_);
  if (auto* state = rlState(states_)) {
    state->setLowlevelOutputMode(lowlevel_output_mode_);
  }
}

void StateMachine::update(RobotState& robot_state, const rclcpp::Time& time,
                          const rclcpp::Duration& period) {
  if (num_joints_ == 0 && !robot_state.joints.empty()) {
    num_joints_ = robot_state.joints.size();
  }
  if (first_update_ && current_state_obj_) {
    current_state_obj_->enter(robot_state, time);
    first_update_ = false;
  }

  const ControllerState next_state = handleStateTransition(time);
  if (next_state != current_state_) {
    changeState(next_state, robot_state, time);
  }
  if (current_state_obj_) {
    current_state_obj_->run(robot_state, time, period);
  }
}

ControllerState StateMachine::handleStateTransition(const rclcpp::Time& time) {
  ControllerState target = target_state_.load();
  if (current_state_ == ControllerState::INIT) {
    if ((time - state_entry_time_).seconds() <= 0.01) {
      return current_state_;
    }
    if (target == ControllerState::INIT) {
      target = ControllerState::IDLE;
      target_state_.store(target);
    }
  }
  return target;
}

void StateMachine::changeState(ControllerState new_state, const RobotState& robot_state,
                               const rclcpp::Time& time) {
  if (new_state == current_state_) {
    return;
  }
  const auto next = states_.find(new_state);
  if (next == states_.end()) {
    RCLCPP_ERROR(logger_, "State object not found for state %d", static_cast<int>(new_state));
    return;
  }
  if (current_state_obj_) {
    current_state_obj_->exit(robot_state, time);
  }

  const ControllerState old_state = current_state_;
  current_state_ = new_state;
  current_state_obj_ = next->second.get();
  state_entry_time_ = time;
  RCLCPP_INFO(logger_, "State transition: %s -> %s", getStateName(old_state).c_str(),
              getStateName(new_state).c_str());
  current_state_obj_->enter(robot_state, time);
}

std::string StateMachine::getStateName(ControllerState state) const {
  switch (state) {
    case ControllerState::INIT:
      return "INIT";
    case ControllerState::IDLE:
      return "IDLE";
    case ControllerState::PREPARE:
      return "PREPARE";
    case ControllerState::RL:
      return "RL";
    default:
      return "UNKNOWN";
  }
}

std::string StateMachine::getStateName() const { return getStateName(current_state_); }

void StateMachine::setTargetState(ControllerState target_state) {
  target_state_.store(target_state);
}

void StateMachine::reset(RobotState& robot_state, const rclcpp::Time& time) {
  if (!first_update_ && current_state_obj_) {
    current_state_obj_->exit(robot_state, time);
  }
  stopRLInference();
  current_state_ = ControllerState::INIT;
  target_state_.store(ControllerState::IDLE);
  state_entry_time_ = time;
  current_state_obj_ = states_.at(ControllerState::INIT).get();
  first_update_ = true;
}

bool StateMachine::initializeRLInference(const std::string& model_path,
                                         int inference_frequency_hz) {
  std::lock_guard<std::mutex> lock(rl_inference_mutex_);
  if (rl_inference_ && rl_inference_->isInitialized()) {
    RCLCPP_WARN(logger_, "ONNX Runtime inference is already initialized");
    return true;
  }

  auto inference = std::make_unique<ONNXRuntimeInference>(logger_);
  if (!inference->initialize(model_path, inference_frequency_hz)) {
    return false;
  }
  rl_inference_ = std::move(inference);
  return true;
}

void StateMachine::startRLInference() {
  std::lock_guard<std::mutex> lock(rl_inference_mutex_);
  if (rl_inference_) {
    rl_inference_->start();
  }
}

void StateMachine::stopRLInference() {
  std::lock_guard<std::mutex> lock(rl_inference_mutex_);
  if (rl_inference_) {
    rl_inference_->stop();
  }
}

void StateMachine::resetRLRuntimeState() {
  std::lock_guard<std::mutex> lock(rl_inference_mutex_);
  if (rl_inference_) {
    rl_inference_->stop();
    if (current_state_ == ControllerState::RL) {
      rl_inference_->start();
    }
  }
}

bool StateMachine::isRLInferenceInitialized() const {
  std::lock_guard<std::mutex> lock(rl_inference_mutex_);
  return rl_inference_ && rl_inference_->isInitialized();
}

bool StateMachine::isRLInferenceRunning() const {
  std::lock_guard<std::mutex> lock(rl_inference_mutex_);
  return rl_inference_ && rl_inference_->isRunning();
}

bool StateMachine::setRLInput(const std::vector<float>& input_data) {
  std::lock_guard<std::mutex> lock(rl_inference_mutex_);
  return rl_inference_ && rl_inference_->setInput(input_data);
}

bool StateMachine::getRLOutput(std::vector<float>& output_data) {
  std::lock_guard<std::mutex> lock(rl_inference_mutex_);
  return rl_inference_ && rl_inference_->getOutput(output_data);
}

size_t StateMachine::getRLInputSize() const {
  std::lock_guard<std::mutex> lock(rl_inference_mutex_);
  return rl_inference_ ? rl_inference_->getInputElementCount() : 0;
}

void StateMachine::setInferenceFrequency(double frequency_hz) {
  if (auto* state = rlState(states_)) {
    state->setInferenceFrequency(frequency_hz);
  }
}

void StateMachine::setJointParams(const std::vector<double>& stiffness,
                                  const std::vector<double>& damping,
                                  const std::vector<double>& action_scale,
                                  const std::vector<double>& output_max,
                                  const std::vector<double>& output_min,
                                  const std::vector<double>& bias,
                                  const std::vector<double>& default_dof_pos) {
  joint_bias_ = bias;
  if (auto* state = rlState(states_)) {
    state->setJointParams(stiffness, damping, action_scale, output_max, output_min, bias,
                          default_dof_pos);
  }
}

void StateMachine::setNoiseEnabled(bool enabled) {
  if (auto* state = rlState(states_)) {
    state->setNoiseEnabled(enabled);
  }
}

void StateMachine::setNoiseParams(const std::vector<double>& imu_gyro_stddev,
                                  const std::vector<double>& imu_accel_stddev,
                                  double joint_position_stddev, double joint_velocity_stddev) {
  imu_gyro_noise_stddev_ = imu_gyro_stddev;
  imu_accel_noise_stddev_ = imu_accel_stddev;
  joint_position_noise_stddev_ = joint_position_stddev;
  joint_velocity_noise_stddev_ = joint_velocity_stddev;
}

void StateMachine::setObservationDelaySteps(int delay_steps) {
  if (auto* state = rlState(states_)) {
    state->setObservationDelaySteps(delay_steps);
  }
}

void StateMachine::setObservationDelayEnabled(bool enabled) {
  if (auto* state = rlState(states_)) {
    state->setObservationDelayEnabled(enabled);
  }
}

void StateMachine::setPolicyInputParams(
    const std::vector<double>& cmd_scale, const std::vector<double>& cmd_max,
    const std::vector<double>& cmd_min, const std::vector<double>& ang_vel_scale,
    const std::vector<double>& ang_vel_max, const std::vector<double>& ang_vel_min,
    const std::vector<double>& gravity_scale, const std::vector<double>& gravity_max,
    const std::vector<double>& gravity_min, const std::vector<double>& joint_pos_scale,
    const std::vector<double>& joint_pos_max, const std::vector<double>& joint_pos_min,
    const std::vector<double>& joint_vel_scale, const std::vector<double>& joint_vel_max,
    const std::vector<double>& joint_vel_min, const std::vector<double>& wheel_vel_scale,
    const std::vector<double>& wheel_vel_max, const std::vector<double>& wheel_vel_min,
    const std::vector<double>& action_scale, const std::vector<double>& action_max,
    const std::vector<double>& action_min) {
  if (auto* state = rlState(states_)) {
    state->setPolicyInputParams(cmd_scale, cmd_max, cmd_min, ang_vel_scale, ang_vel_max,
                                ang_vel_min, gravity_scale, gravity_max, gravity_min,
                                joint_pos_scale, joint_pos_max, joint_pos_min, joint_vel_scale,
                                joint_vel_max, joint_vel_min, wheel_vel_scale, wheel_vel_max,
                                wheel_vel_min, action_scale, action_max, action_min);
  }
}

void StateMachine::setPrepareParams(const std::vector<double>& target_pos,
                                    const std::vector<double>& kp, const std::vector<double>& kd,
                                    double max_velocity_rad_per_sec) {
  prepare_target_pos_ = target_pos;
  prepare_kp_ = kp;
  prepare_kd_ = kd;
  prepare_max_velocity_ = max_velocity_rad_per_sec;
}

void StateMachine::setPrintInferenceTime(bool print) {
  std::lock_guard<std::mutex> lock(rl_inference_mutex_);
  if (rl_inference_) {
    rl_inference_->setPrintInferenceTime(print);
  }
}

void StateMachine::setPublishNetworkIO(bool publish) {
  if (auto* state = rlState(states_)) {
    state->setPublishNetworkIO(publish);
  }
}

void StateMachine::setLowlevelOutputMode(const std::string& mode) {
  lowlevel_output_mode_ = mode;
  if (auto* state = rlState(states_)) {
    state->setLowlevelOutputMode(mode);
  }
}

void StateMachine::setActionFilterConfig(const std::string& type, int window, double alpha) {
  if (auto* state = rlState(states_)) {
    state->setActionFilterConfig(type, window, alpha);
  }
}

}  // namespace robot_locomotion
