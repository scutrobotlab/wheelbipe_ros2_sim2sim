// Copyright (c) 2025 SCUTRobotLab
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef FSM__STATES__STATE_RL_HPP_
#define FSM__STATES__STATE_RL_HPP_

#include <array>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "fsm/state_base.hpp"
#include "robot_state/robot_state.hpp"
#include "utils/delay_buffer.hpp"
#include "utils/noise_generator.hpp"

namespace robot_locomotion {

struct ModelParams {
  std::array<float, 8> joint_stiffness{};
  std::array<float, 8> joint_damping{};
  std::array<float, 8> joint_action_scale{};
  std::array<float, 8> joint_output_max{};
  std::array<float, 8> joint_output_min{};
  std::array<float, 8> joint_bias{};
  std::array<float, 8> default_dof_pos{};
};

struct PolicyInputParams {
  std::vector<float> cmd_scale, cmd_max, cmd_min;
  std::vector<float> ang_vel_scale, ang_vel_max, ang_vel_min;
  std::vector<float> gravity_scale, gravity_max, gravity_min;
  std::vector<float> joint_pos_scale, joint_pos_max, joint_pos_min;
  std::vector<float> joint_vel_scale, joint_vel_max, joint_vel_min;
  std::vector<float> wheel_vel_scale, wheel_vel_max, wheel_vel_min;
  std::vector<float> action_scale, action_max, action_min;
};

enum class ActionFilterType { NONE = 0, MOVING_AVG, LOWPASS };

/// RL state for the fixed 35-observation, 6-action WheelBipe policy contract.
class StateRL : public StateBase {
 public:
  StateRL(StateMachine* state_machine, rclcpp::Logger logger,
          rclcpp_lifecycle::LifecycleNode::SharedPtr node);
  ~StateRL() override = default;

  void enter(const RobotState& robot_state, const rclcpp::Time& time) override;
  void run(RobotState& robot_state, const rclcpp::Time& time,
           const rclcpp::Duration& period) override;
  void exit(const RobotState& robot_state, const rclcpp::Time& time) override;
  std::string getName() const override { return "RL"; }

  void setJointParams(const std::vector<double>& stiffness, const std::vector<double>& damping,
                      const std::vector<double>& action_scale,
                      const std::vector<double>& output_max, const std::vector<double>& output_min,
                      const std::vector<double>& bias, const std::vector<double>& default_dof_pos);
  void setInferenceFrequency(double frequency_hz);
  void setNoiseEnabled(bool enabled) { enable_noise_ = enabled; }
  void setObservationDelaySteps(int delay_steps);
  void setObservationDelayEnabled(bool enabled) { enable_delay_ = enabled; }
  void setPolicyInputParams(
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
      const std::vector<double>& action_min);
  void setActionFilterConfig(const std::string& type, int window, double alpha);
  void setPublishNetworkIO(bool publish) { publish_network_io_ = publish; }
  void setLowlevelOutputMode(const std::string& mode) { lowlevel_output_mode_ = mode; }

 private:
  std::vector<float> buildObservation(const RobotState& robot_state);
  bool shouldInfer(const rclcpp::Time& time);
  double filterAction(size_t index, double raw_action);
  void applyLowlevelControl(RobotState& robot_state);
  void failSafe(RobotState& robot_state, const std::string& reason);
  void resetPolicyRuntimeState();

  ModelParams params_;
  PolicyInputParams policy_input_params_;
  std::array<float, 6> last_actions_{};
  std::array<double, 8> desired_pos_{};
  std::array<double, 8> torque_{};

  double inference_frequency_hz_ = 50.0;
  rclcpp::Time last_inference_time_;
  bool last_inference_time_initialized_ = false;

  ActionFilterType action_filter_type_ = ActionFilterType::NONE;
  int action_filter_window_ = 1;
  double action_filter_alpha_ = 0.2;
  std::array<std::deque<double>, 6> action_ma_buffers_;
  std::array<double, 6> action_lp_last_output_{};

  ObservationDelayBuffer observation_delay_buffer_;
  bool enable_delay_ = false;
  bool enable_noise_ = false;
  NoiseGenerator noise_generator_;

  bool publish_network_io_ = true;
  std::string lowlevel_output_mode_ = "hardware_pd_vel";
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr network_input_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr network_output_publisher_;
};

}  // namespace robot_locomotion

#endif  // FSM__STATES__STATE_RL_HPP_
