// Copyright (c) 2025 SCUTRobotLab
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef FSM__STATE_MACHINE_HPP_
#define FSM__STATE_MACHINE_HPP_

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace robot_locomotion {

struct RobotState;
class ONNXRuntimeInference;
class StateBase;

enum class ControllerState { INIT = 0, IDLE, PREPARE, RL };

/// INIT -> IDLE/PREPARE/RL state machine shared by simulation and hardware adapters.
class StateMachine {
 public:
  explicit StateMachine(rclcpp::Logger logger,
                        rclcpp_lifecycle::LifecycleNode::SharedPtr node = nullptr);
  ~StateMachine();

  void update(RobotState& robot_state, const rclcpp::Time& time, const rclcpp::Duration& period);
  ControllerState getCurrentState() const { return current_state_; }
  ControllerState getTargetState() const { return target_state_.load(); }
  std::string getStateName() const;
  std::string getStateName(ControllerState state) const;
  void setTargetState(ControllerState target_state);
  void reset(RobotState& robot_state, const rclcpp::Time& time);

  bool initializeRLInference(const std::string& model_path, int inference_frequency_hz);
  void startRLInference();
  void stopRLInference();
  void resetRLRuntimeState();
  bool isRLInferenceInitialized() const;
  bool isRLInferenceRunning() const;
  bool setRLInput(const std::vector<float>& input_data);
  bool getRLOutput(std::vector<float>& output_data);
  size_t getRLInputSize() const;

  void setInferenceFrequency(double frequency_hz);
  void setJointParams(const std::vector<double>& stiffness, const std::vector<double>& damping,
                      const std::vector<double>& action_scale,
                      const std::vector<double>& output_max, const std::vector<double>& output_min,
                      const std::vector<double>& bias, const std::vector<double>& default_dof_pos);
  void setNoiseEnabled(bool enabled);
  void setNoiseParams(const std::vector<double>& imu_gyro_stddev,
                      const std::vector<double>& imu_accel_stddev, double joint_position_stddev,
                      double joint_velocity_stddev);
  void setObservationDelaySteps(int delay_steps);
  void setObservationDelayEnabled(bool enabled);
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

  void setPrepareParams(const std::vector<double>& target_pos, const std::vector<double>& kp,
                        const std::vector<double>& kd, double max_velocity_rad_per_sec);
  const std::vector<double>& getPrepareTargetPos() const { return prepare_target_pos_; }
  const std::vector<double>& getPrepareKp() const { return prepare_kp_; }
  const std::vector<double>& getPrepareKd() const { return prepare_kd_; }
  double getPrepareMaxVelocity() const { return prepare_max_velocity_; }
  const std::vector<double>& getJointBias() const { return joint_bias_; }

  void setPrintInferenceTime(bool print);
  void setPublishNetworkIO(bool publish);
  void setLowlevelOutputMode(const std::string& mode);
  const std::string& getLowlevelOutputMode() const { return lowlevel_output_mode_; }
  void setActionFilterConfig(const std::string& type, int window, double alpha);

  std::vector<double> getIMUGyroNoiseStddev() const { return imu_gyro_noise_stddev_; }
  std::vector<double> getIMUAccelNoiseStddev() const { return imu_accel_noise_stddev_; }
  double getJointPositionNoiseStddev() const { return joint_position_noise_stddev_; }
  double getJointVelocityNoiseStddev() const { return joint_velocity_noise_stddev_; }

 private:
  ControllerState handleStateTransition(const rclcpp::Time& time);
  void changeState(ControllerState new_state, const RobotState& robot_state,
                   const rclcpp::Time& time);
  void initializeStates();

  ControllerState current_state_ = ControllerState::INIT;
  std::atomic<ControllerState> target_state_{ControllerState::IDLE};
  rclcpp::Logger logger_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Time state_entry_time_;
  size_t num_joints_ = 0;
  bool first_update_ = true;
  std::map<ControllerState, std::unique_ptr<StateBase>> states_;
  StateBase* current_state_obj_ = nullptr;

  std::unique_ptr<ONNXRuntimeInference> rl_inference_;
  mutable std::mutex rl_inference_mutex_;

  std::vector<double> prepare_target_pos_;
  std::vector<double> prepare_kp_;
  std::vector<double> prepare_kd_;
  double prepare_max_velocity_ = 0.5;
  std::vector<double> joint_bias_;

  std::vector<double> imu_gyro_noise_stddev_{0.01, 0.01, 0.01};
  std::vector<double> imu_accel_noise_stddev_{0.01, 0.01, 0.01};
  double joint_position_noise_stddev_ = 0.01;
  double joint_velocity_noise_stddev_ = 0.01;
  std::string lowlevel_output_mode_ = "hardware_pd_vel";
};

}  // namespace robot_locomotion

#endif  // FSM__STATE_MACHINE_HPP_
