// Copyright (c) 2025 SCUTRobotLab
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "fsm/states/state_rl.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "fsm/state_machine.hpp"

namespace robot_locomotion {
namespace {

constexpr size_t kObservationSize = 35;
constexpr size_t kActionSize = 6;
constexpr size_t kLegJointCount = 4;
constexpr std::array<float, 7> kNormalControlMode = {1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};

float scaleClamp(float value, const std::vector<float>& scale, const std::vector<float>& maximum,
                 const std::vector<float>& minimum, size_t index) {
  if (index < scale.size()) {
    value *= scale[index];
  }
  if (index < maximum.size()) {
    value = std::min(value, maximum[index]);
  }
  if (index < minimum.size()) {
    value = std::max(value, minimum[index]);
  }
  return value;
}

std::vector<float> toFloat(const std::vector<double>& values) {
  return std::vector<float>(values.begin(), values.end());
}

}  // namespace

StateRL::StateRL(StateMachine* state_machine, rclcpp::Logger logger,
                 rclcpp_lifecycle::LifecycleNode::SharedPtr node)
    : StateBase(state_machine, logger), node_(node) {
  params_.joint_action_scale.fill(1.0F);
}

void StateRL::enter(const RobotState& robot_state, const rclcpp::Time& time) {
  (void)robot_state;
  (void)time;
  resetPolicyRuntimeState();
  if (publish_network_io_ && node_) {
    if (!network_input_publisher_) {
      network_input_publisher_ =
          node_->create_publisher<std_msgs::msg::Float64MultiArray>("~/rl_network_input", 10);
    }
    if (!network_output_publisher_) {
      network_output_publisher_ =
          node_->create_publisher<std_msgs::msg::Float64MultiArray>("~/rl_network_output", 10);
    }
  }
  if (state_machine_) {
    state_machine_->startRLInference();
  }
  RCLCPP_INFO(logger_,
              "Entering RL state: ONNX Runtime CPU, 35 observations (normal-only), 6 actions");
}

void StateRL::exit(const RobotState& robot_state, const rclcpp::Time& time) {
  (void)robot_state;
  (void)time;
  if (state_machine_) {
    state_machine_->stopRLInference();
  }
  resetPolicyRuntimeState();
  RCLCPP_INFO(logger_, "Exiting RL state");
}

void StateRL::run(RobotState& robot_state, const rclcpp::Time& time,
                  const rclcpp::Duration& period) {
  (void)period;
  if (!state_machine_ || !state_machine_->isRLInferenceInitialized()) {
    failSafe(robot_state, "policy session is not initialized");
    return;
  }

  const std::vector<float> observation = buildObservation(robot_state);
  if (observation.size() != kObservationSize ||
      !std::all_of(observation.begin(), observation.end(),
                   [](float value) { return std::isfinite(value); })) {
    failSafe(robot_state, "policy observation is not a finite 35D vector");
    return;
  }

  if (shouldInfer(time)) {
    if (publish_network_io_ && network_input_publisher_) {
      std_msgs::msg::Float64MultiArray message;
      message.data.assign(observation.begin(), observation.end());
      network_input_publisher_->publish(message);
    }

    if (!state_machine_->setRLInput(observation)) {
      failSafe(robot_state, "ONNX Runtime rejected the policy input");
      return;
    }

    std::vector<float> model_output;
    if (!state_machine_->getRLOutput(model_output) || model_output.size() != kActionSize ||
        !std::all_of(model_output.begin(), model_output.end(),
                     [](float value) { return std::isfinite(value); })) {
      failSafe(robot_state, "policy output is not a finite 6D vector");
      return;
    }

    std::array<double, kActionSize> filtered_action{};
    std::array<double, kActionSize> scaled_action{};
    for (size_t index = 0; index < kActionSize; ++index) {
      const double filtered = filterAction(index, model_output[index]);
      filtered_action[index] = filtered;
      scaled_action[index] =
          params_.joint_action_scale[index] * filtered + params_.default_dof_pos[index];
      if (!std::isfinite(filtered) || !std::isfinite(scaled_action[index])) {
        failSafe(robot_state, "scaled policy action is not finite");
        return;
      }
    }

    for (size_t index = 0; index < kActionSize; ++index) {
      last_actions_[index] = static_cast<float>(filtered_action[index]);
      desired_pos_[index] = scaled_action[index];
    }

    if (publish_network_io_ && network_output_publisher_) {
      std_msgs::msg::Float64MultiArray message;
      message.data.assign(model_output.begin(), model_output.end());
      network_output_publisher_->publish(message);
    }
  }

  applyLowlevelControl(robot_state);
}

std::vector<float> StateRL::buildObservation(const RobotState& robot_state) {
  std::vector<float> command_observation;
  command_observation.reserve(4);
  for (size_t index = 0; index < robot_state.command.cmd_vel.size(); ++index) {
    command_observation.push_back(scaleClamp(
        static_cast<float>(robot_state.command.cmd_vel[index]), policy_input_params_.cmd_scale,
        policy_input_params_.cmd_max, policy_input_params_.cmd_min, index));
  }
  command_observation.push_back(
      scaleClamp(static_cast<float>(robot_state.command.cmd_height), policy_input_params_.cmd_scale,
                 policy_input_params_.cmd_max, policy_input_params_.cmd_min, 3));

  std::vector<float> sensor_observation;
  sensor_observation.reserve(18);
  for (size_t index = 0; index < 3; ++index) {
    sensor_observation.push_back(
        scaleClamp(static_cast<float>(robot_state.body_state.ang_vel_b[index]),
                   policy_input_params_.ang_vel_scale, policy_input_params_.ang_vel_max,
                   policy_input_params_.ang_vel_min, index));
  }

  const Vec3<double> projected_gravity =
      robot_state.body_state.rotation_w2b * Vec3<double>(0.0, 0.0, -1.0);
  for (size_t index = 0; index < 3; ++index) {
    sensor_observation.push_back(
        scaleClamp(static_cast<float>(projected_gravity[index]), policy_input_params_.gravity_scale,
                   policy_input_params_.gravity_max, policy_input_params_.gravity_min, index));
  }

  size_t policy_joint_index = 0;
  for (const auto& joint : robot_state.joints) {
    if (joint.name.find("spring") != std::string::npos) {
      continue;
    }
    const float position =
        joint.name.find("wheel") == std::string::npos ? static_cast<float>(joint.position) : 0.0F;
    sensor_observation.push_back(scaleClamp(
        position, policy_input_params_.joint_pos_scale, policy_input_params_.joint_pos_max,
        policy_input_params_.joint_pos_min, policy_joint_index));
    ++policy_joint_index;
  }

  policy_joint_index = 0;
  for (const auto& joint : robot_state.joints) {
    if (joint.name.find("spring") != std::string::npos) {
      continue;
    }
    const float velocity = static_cast<float>(joint.velocity);
    if (joint.name.find("wheel") != std::string::npos) {
      const size_t wheel_index =
          policy_joint_index >= kLegJointCount ? policy_joint_index - kLegJointCount : 0;
      sensor_observation.push_back(scaleClamp(velocity, policy_input_params_.wheel_vel_scale,
                                              policy_input_params_.wheel_vel_max,
                                              policy_input_params_.wheel_vel_min, wheel_index));
    } else {
      sensor_observation.push_back(scaleClamp(
          velocity, policy_input_params_.joint_vel_scale, policy_input_params_.joint_vel_max,
          policy_input_params_.joint_vel_min, policy_joint_index));
    }
    ++policy_joint_index;
  }

  if (enable_noise_ && state_machine_) {
    const auto gyro_noise = state_machine_->getIMUGyroNoiseStddev();
    const auto gravity_noise = state_machine_->getIMUAccelNoiseStddev();
    for (size_t index = 0; index < 3 && index < gyro_noise.size(); ++index) {
      sensor_observation[index] +=
          static_cast<float>(noise_generator_.generateGaussianNoise(0.0, gyro_noise[index]));
    }
    for (size_t index = 0; index < 3 && index < gravity_noise.size(); ++index) {
      sensor_observation[index + 3] +=
          static_cast<float>(noise_generator_.generateGaussianNoise(0.0, gravity_noise[index]));
    }
    for (size_t index = 6; index < 12 && index < sensor_observation.size(); ++index) {
      sensor_observation[index] += static_cast<float>(noise_generator_.generateGaussianNoise(
          0.0, state_machine_->getJointPositionNoiseStddev()));
    }
    for (size_t index = 12; index < 16 && index < sensor_observation.size(); ++index) {
      sensor_observation[index] += static_cast<float>(noise_generator_.generateGaussianNoise(
          0.0, state_machine_->getJointVelocityNoiseStddev()));
    }
  }

  const std::vector<float> delayed_sensor_observation =
      enable_delay_ ? observation_delay_buffer_.pushAndGet(sensor_observation) : sensor_observation;

  std::vector<float> observation;
  observation.reserve(kObservationSize);
  observation.insert(observation.end(), command_observation.begin(), command_observation.end());
  observation.insert(observation.end(), delayed_sensor_observation.begin(),
                     delayed_sensor_observation.end());
  for (size_t index = 0; index < last_actions_.size(); ++index) {
    observation.push_back(scaleClamp(last_actions_[index], policy_input_params_.action_scale,
                                     policy_input_params_.action_max,
                                     policy_input_params_.action_min, index));
  }
  observation.insert(observation.end(), kNormalControlMode.begin(), kNormalControlMode.end());
  return observation;
}

bool StateRL::shouldInfer(const rclcpp::Time& time) {
  if (!last_inference_time_initialized_ || time < last_inference_time_) {
    last_inference_time_ = time;
    last_inference_time_initialized_ = true;
    return true;
  }
  const double period = 1.0 / inference_frequency_hz_;
  if ((time - last_inference_time_).seconds() + 1.0e-9 < period) {
    return false;
  }
  last_inference_time_ = time;
  return true;
}

void StateRL::applyLowlevelControl(RobotState& robot_state) {
  const size_t joint_count = std::min(robot_state.joints.size(), params_.joint_bias.size());
  for (size_t index = 0; index < joint_count; ++index) {
    if (index < kLegJointCount) {
      torque_[index] = params_.joint_stiffness[index] *
                           (desired_pos_[index] - robot_state.joints[index].position) -
                       params_.joint_damping[index] * robot_state.joints[index].velocity;
    } else if (index < kActionSize) {
      if (lowlevel_output_mode_ == "hardware_pd_vel") {
        torque_[index] = 0.0;
      } else if (lowlevel_output_mode_ == "hardware_pd") {
        torque_[index] = desired_pos_[index];
      } else {
        torque_[index] =
            desired_pos_[index] - params_.joint_damping[index] * robot_state.joints[index].velocity;
      }
    } else {
      desired_pos_[index] = robot_state.joints[index].position;
      torque_[index] = 0.0;
    }

    const double output_min = params_.joint_output_min[index];
    const double output_max = params_.joint_output_max[index];
    if (!std::isfinite(torque_[index]) || !std::isfinite(output_min) ||
        !std::isfinite(output_max) || output_min > output_max) {
      failSafe(robot_state, "low-level control output is invalid");
      return;
    }
    torque_[index] = std::clamp(torque_[index], output_min, output_max);

    robot_state.joints[index].output_position = desired_pos_[index];
    robot_state.joints[index].output_velocity =
        index >= kLegJointCount && index < kActionSize && lowlevel_output_mode_ == "hardware_pd_vel"
            ? desired_pos_[index]
            : 0.0;
    robot_state.joints[index].output_torque =
        torque_[index] + static_cast<double>(params_.joint_bias[index]);
  }
}

void StateRL::failSafe(RobotState& robot_state, const std::string& reason) {
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  RCLCPP_ERROR_THROTTLE(logger_, clock, 1000, "RL safety stop: %s", reason.c_str());
  resetPolicyRuntimeState();
  const size_t count = std::min(robot_state.joints.size(), desired_pos_.size());
  for (size_t index = 0; index < count; ++index) {
    desired_pos_[index] = index < kLegJointCount ? robot_state.joints[index].position : 0.0;
    torque_[index] = 0.0;
    robot_state.joints[index].output_position = robot_state.joints[index].position;
    robot_state.joints[index].output_velocity = 0.0;
    robot_state.joints[index].output_torque = 0.0;
    robot_state.joints[index].output_kp = 0.0;
    robot_state.joints[index].output_kd = 0.0;
  }
  if (state_machine_) {
    state_machine_->setTargetState(ControllerState::IDLE);
  }
}

void StateRL::resetPolicyRuntimeState() {
  last_actions_.fill(0.0F);
  desired_pos_.fill(0.0);
  torque_.fill(0.0);
  last_inference_time_initialized_ = false;
  observation_delay_buffer_.clear();
  for (auto& buffer : action_ma_buffers_) {
    buffer.clear();
  }
  action_lp_last_output_.fill(0.0);
}

void StateRL::setInferenceFrequency(double frequency_hz) {
  if (std::isfinite(frequency_hz) && frequency_hz > 0.0) {
    inference_frequency_hz_ = frequency_hz;
    last_inference_time_initialized_ = false;
  }
}

void StateRL::setObservationDelaySteps(int delay_steps) {
  observation_delay_buffer_.setDelaySteps(static_cast<size_t>(std::max(0, delay_steps)));
}

void StateRL::setActionFilterConfig(const std::string& type, int window, double alpha) {
  if (type == "none") {
    action_filter_type_ = ActionFilterType::NONE;
  } else if (type == "moving_avg") {
    action_filter_type_ = ActionFilterType::MOVING_AVG;
  } else if (type == "lowpass") {
    action_filter_type_ = ActionFilterType::LOWPASS;
  } else {
    throw std::invalid_argument("Unsupported RL action filter type: " + type);
  }
  if (window < 1) {
    throw std::invalid_argument("RL action filter window must be positive");
  }
  if (!std::isfinite(alpha) || alpha < 0.0 || alpha > 1.0) {
    throw std::invalid_argument("RL action filter alpha must be within [0, 1]");
  }
  action_filter_window_ = window;
  action_filter_alpha_ = alpha;
  for (auto& buffer : action_ma_buffers_) {
    buffer.clear();
  }
  action_lp_last_output_.fill(0.0);
}

double StateRL::filterAction(size_t index, double raw_action) {
  if (index >= kActionSize) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (action_filter_type_ == ActionFilterType::MOVING_AVG) {
    auto& buffer = action_ma_buffers_[index];
    buffer.push_back(raw_action);
    while (static_cast<int>(buffer.size()) > action_filter_window_) {
      buffer.pop_front();
    }
    double sum = 0.0;
    for (double value : buffer) {
      sum += value;
    }
    return sum / static_cast<double>(buffer.size());
  }
  if (action_filter_type_ == ActionFilterType::LOWPASS) {
    const double filtered = action_filter_alpha_ * raw_action +
                            (1.0 - action_filter_alpha_) * action_lp_last_output_[index];
    action_lp_last_output_[index] = filtered;
    return filtered;
  }
  return raw_action;
}

void StateRL::setJointParams(const std::vector<double>& stiffness,
                             const std::vector<double>& damping,
                             const std::vector<double>& action_scale,
                             const std::vector<double>& output_max,
                             const std::vector<double>& output_min, const std::vector<double>& bias,
                             const std::vector<double>& default_dof_pos) {
  constexpr size_t kExpectedJointCount = 8;
  if (stiffness.size() != kExpectedJointCount || damping.size() != kExpectedJointCount ||
      action_scale.size() != kExpectedJointCount || output_max.size() != kExpectedJointCount ||
      output_min.size() != kExpectedJointCount || bias.size() != kExpectedJointCount ||
      default_dof_pos.size() != kExpectedJointCount) {
    throw std::invalid_argument("RL joint parameter arrays must each contain exactly 8 values");
  }
  for (size_t index = 0; index < kExpectedJointCount; ++index) {
    params_.joint_stiffness[index] = static_cast<float>(stiffness[index]);
    params_.joint_damping[index] = static_cast<float>(damping[index]);
    params_.joint_action_scale[index] = static_cast<float>(action_scale[index]);
    params_.joint_output_max[index] = static_cast<float>(output_max[index]);
    params_.joint_output_min[index] = static_cast<float>(output_min[index]);
    params_.joint_bias[index] = static_cast<float>(bias[index]);
    params_.default_dof_pos[index] = static_cast<float>(default_dof_pos[index]);
  }
}

void StateRL::setPolicyInputParams(
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
  policy_input_params_.cmd_scale = toFloat(cmd_scale);
  policy_input_params_.cmd_max = toFloat(cmd_max);
  policy_input_params_.cmd_min = toFloat(cmd_min);
  policy_input_params_.ang_vel_scale = toFloat(ang_vel_scale);
  policy_input_params_.ang_vel_max = toFloat(ang_vel_max);
  policy_input_params_.ang_vel_min = toFloat(ang_vel_min);
  policy_input_params_.gravity_scale = toFloat(gravity_scale);
  policy_input_params_.gravity_max = toFloat(gravity_max);
  policy_input_params_.gravity_min = toFloat(gravity_min);
  policy_input_params_.joint_pos_scale = toFloat(joint_pos_scale);
  policy_input_params_.joint_pos_max = toFloat(joint_pos_max);
  policy_input_params_.joint_pos_min = toFloat(joint_pos_min);
  policy_input_params_.joint_vel_scale = toFloat(joint_vel_scale);
  policy_input_params_.joint_vel_max = toFloat(joint_vel_max);
  policy_input_params_.joint_vel_min = toFloat(joint_vel_min);
  policy_input_params_.wheel_vel_scale = toFloat(wheel_vel_scale);
  policy_input_params_.wheel_vel_max = toFloat(wheel_vel_max);
  policy_input_params_.wheel_vel_min = toFloat(wheel_vel_min);
  policy_input_params_.action_scale = toFloat(action_scale);
  policy_input_params_.action_max = toFloat(action_max);
  policy_input_params_.action_min = toFloat(action_min);
}

}  // namespace robot_locomotion
