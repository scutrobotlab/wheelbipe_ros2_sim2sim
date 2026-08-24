// Copyright (c) 2025 Sangtaek Lee
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_HPP_

#include <Eigen/Dense>
#include <cstddef>
#include <deque>
#include <string>
#include <vector>

#include "control_toolbox/pid.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_limits/joint_limits.hpp"
#include "mujoco_ros2_control/mujoco_system_interface.hpp"

namespace mujoco_ros2_control {
constexpr char PARAM_KP[]{"_kp"};
constexpr char PARAM_KI[]{"_ki"};
constexpr char PARAM_KD[]{"_kd"};
constexpr char PARAM_I_MAX[]{"_i_max"};
constexpr char PARAM_I_MIN[]{"_i_min"};

class MujocoSystem : public MujocoSystemInterface {
 public:
  MujocoSystem();
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

  bool init_sim(mjModel* mujoco_model, mjData* mujoco_data, const urdf::Model& urdf_model,
                const hardware_interface::HardwareInfo& hardware_info) override;

  struct JointState {
    std::string name;
    double position;
    double velocity;
    double effort;
    double position_command;
    double velocity_command;
    double effort_command;
    // Additional command gains used by the WheelBipe controller.
    double kp{0.0};
    double kd{0.0};
    double min_effort_command;
    double max_effort_command;
    control_toolbox::Pid position_pid;
    control_toolbox::Pid velocity_pid;
    bool is_position_control_enabled{false};
    bool is_velocity_control_enabled{false};
    bool is_effort_control_enabled{false};
    bool is_pid_enabled{false};
    joint_limits::JointLimits joint_limits;
    bool is_mimic{false};
    int mimicked_joint_index;
    double mimic_multiplier;
    int mj_joint_type;
    int mj_pos_adr;
    int mj_vel_adr;
    double last_wrap_warn_time{-1.0};

    // Optional actuator dynamics between the requested/clipped joint torque and
    // the torque applied to MuJoCo. This models the real motor FOC response.
    bool actuator_model_enabled{false};
    double actuator_time_constant_s{0.0};
    double actuator_transport_delay_s{0.0};
    double actuator_dc_gain{1.0};
    double actuator_natural_frequency_hz{0.0};
    double actuator_damping_ratio{1.0};
    double actuator_torque_slew_rate_nm_s{0.0};
    double actuator_output_limit{0.0};
    double actuator_speed_droop_start_rad_s{0.0};
    double actuator_speed_droop_per_rad_s{0.0};
    double actuator_min_speed_gain{1.0};
    double actuator_speed_gain{1.0};
    double actuator_requested_effort{0.0};
    double actuator_applied_effort{0.0};
    double actuator_applied_effort_rate{0.0};
    bool actuator_model_initialized{false};
    size_t actuator_delay_steps{0};
    std::deque<double> actuator_delay_line;
  };

  template <typename T>
  struct SensorData {
    std::string name;
    T data;
    int mj_sensor_index;
  };

  struct FTSensorData {
    std::string name;
    SensorData<Eigen::Vector3d> force;
    SensorData<Eigen::Vector3d> torque;
  };

  struct IMUSensorData {
    std::string name;
    SensorData<Eigen::Quaternion<double>> orientation;
    SensorData<Eigen::Vector3d> angular_velocity;
    SensorData<Eigen::Vector3d> linear_acceleration;
  };

 private:
  struct GasSpringModel {
    bool enabled{false};
    double q_min{-0.01};
    double q_max{0.06};
    double force_at_q_min{780.0};
    double force_at_q_max{450.0};
    bool manual_passive_enabled{false};
    double manual_viscous_damping{0.0};
    double manual_static_friction{0.0};
    double manual_velocity_epsilon{0.01};
  };

  bool register_joints(const urdf::Model& urdf_model,
                       const hardware_interface::HardwareInfo& hardware_info);
  void register_sensors(const urdf::Model& urdf_model,
                        const hardware_interface::HardwareInfo& hardware_info);
  void configure_joint_actuator_model(JointState& joint_state,
                                      const hardware_interface::ComponentInfo& joint_info);
  double apply_joint_actuator_model(JointState& joint_state, double requested_effort, double dt);
  void configure_gas_spring_model(const hardware_interface::HardwareInfo& hardware_info);
  double gas_spring_model_force(const JointState& joint) const;
  bool is_gas_spring_joint(const JointState& joint) const;
  void reset_runtime_state();
  void set_initial_pose();
  void get_joint_limits(urdf::JointConstSharedPtr urdf_joint,
                        joint_limits::JointLimits& joint_limits);
  control_toolbox::Pid get_pid_gains(const hardware_interface::ComponentInfo& joint_info,
                                     std::string command_interface);
  double clamp(double v, double lo, double hi) const { return (v < lo) ? lo : (hi < v) ? hi : v; }

  std::vector<hardware_interface::StateInterface> state_interfaces_;
  std::vector<hardware_interface::CommandInterface> command_interfaces_;

  std::vector<JointState> joint_states_;
  std::vector<FTSensorData> ft_sensor_data_;
  std::vector<IMUSensorData> imu_sensor_data_;

  mjModel* mj_model_;
  mjData* mj_data_;
  double last_sim_time_{-1.0};

  GasSpringModel gas_spring_model_;

  rclcpp::Logger logger_;
};
}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_HPP_
