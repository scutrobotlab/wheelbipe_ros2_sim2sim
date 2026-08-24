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

#include "mujoco_ros2_control/mujoco_system.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

namespace mujoco_ros2_control {
namespace {

double normalizeAngle(double angle, double modulus) {
  const double half_modulus = modulus / 2.0;
  angle = std::fmod(angle, modulus);
  if (angle > half_modulus) {
    angle -= modulus;
  } else if (angle < -half_modulus) {
    angle += modulus;
  }
  return angle;
}

double angleDifference(double first, double second, double modulus) {
  double difference = first - second;
  const double half_modulus = modulus / 2.0;
  if (difference > half_modulus) {
    difference -= modulus;
  } else if (difference < -half_modulus) {
    difference += modulus;
  }
  return difference;
}

double nonnegativeFiniteGain(double value) {
  if (!std::isfinite(value) || value < 0.0) {
    return 0.0;
  }
  return value;
}

bool parseBool(const std::string& value) {
  return value == "1" || value == "true" || value == "True" || value == "TRUE" || value == "yes" ||
         value == "on";
}

constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kJointWrapWarnMinDeg = 175.0;
constexpr double kJointWrapWarnMaxDeg = 185.0;
constexpr double kJointWrapWarnIntervalSec = 0.2;

bool isFrontRearDriveJoint(const std::string& name) {
  return name == "left_front1_joint" || name == "left_rear1_joint" ||
         name == "right_front1_joint" || name == "right_rear1_joint";
}

bool isNearSigned180Deg(double degrees) {
  const double abs_degrees = std::abs(degrees);
  return abs_degrees >= kJointWrapWarnMinDeg && abs_degrees <= kJointWrapWarnMaxDeg;
}

}  // namespace

MujocoSystem::MujocoSystem() : logger_(rclcpp::get_logger("")) {}

std::vector<hardware_interface::StateInterface> MujocoSystem::export_state_interfaces() {
  return std::move(state_interfaces_);
}

std::vector<hardware_interface::CommandInterface> MujocoSystem::export_command_interfaces() {
  return std::move(command_interfaces_);
}

hardware_interface::return_type MujocoSystem::read(const rclcpp::Time& /* time */,
                                                   const rclcpp::Duration& /* period */) {
  if (last_sim_time_ >= 0.0 && mj_data_->time < last_sim_time_) {
    reset_runtime_state();
  }
  last_sim_time_ = mj_data_->time;

  // Joint states
  for (auto& joint_state : joint_states_) {
    const double position = mj_data_->qpos[joint_state.mj_pos_adr];
    joint_state.position = normalizeAngle(position, 2 * M_PI);
    joint_state.velocity = mj_data_->qvel[joint_state.mj_vel_adr];
    joint_state.effort = mj_data_->qfrc_applied[joint_state.mj_vel_adr];
  }

  // IMU Sensor data
  for (auto& data : imu_sensor_data_) {
    data.orientation.data.w() = mj_data_->sensordata[data.orientation.mj_sensor_index];
    data.orientation.data.x() = mj_data_->sensordata[data.orientation.mj_sensor_index + 1];
    data.orientation.data.y() = mj_data_->sensordata[data.orientation.mj_sensor_index + 2];
    data.orientation.data.z() = mj_data_->sensordata[data.orientation.mj_sensor_index + 3];

    data.angular_velocity.data.x() = mj_data_->sensordata[data.angular_velocity.mj_sensor_index];
    data.angular_velocity.data.y() =
        mj_data_->sensordata[data.angular_velocity.mj_sensor_index + 1];
    data.angular_velocity.data.z() =
        mj_data_->sensordata[data.angular_velocity.mj_sensor_index + 2];

    data.linear_acceleration.data.x() =
        mj_data_->sensordata[data.linear_acceleration.mj_sensor_index];
    data.linear_acceleration.data.y() =
        mj_data_->sensordata[data.linear_acceleration.mj_sensor_index + 1];
    data.linear_acceleration.data.z() =
        mj_data_->sensordata[data.linear_acceleration.mj_sensor_index + 2];
  }

  // FT Sensor data
  for (auto& data : ft_sensor_data_) {
    data.force.data.x() = -mj_data_->sensordata[data.force.mj_sensor_index];
    data.force.data.y() = -mj_data_->sensordata[data.force.mj_sensor_index + 1];
    data.force.data.z() = -mj_data_->sensordata[data.force.mj_sensor_index + 2];

    data.torque.data.x() = -mj_data_->sensordata[data.torque.mj_sensor_index];
    data.torque.data.y() = -mj_data_->sensordata[data.torque.mj_sensor_index + 1];
    data.torque.data.z() = -mj_data_->sensordata[data.torque.mj_sensor_index + 2];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystem::write(const rclcpp::Time& /* time */,
                                                    const rclcpp::Duration& period) {
  if (last_sim_time_ >= 0.0 && mj_data_->time < last_sim_time_) {
    reset_runtime_state();
  }
  last_sim_time_ = mj_data_->time;

  double actuator_dt = period.seconds();
  if (mj_model_ != nullptr && std::isfinite(mj_model_->opt.timestep) &&
      mj_model_->opt.timestep > 0.0) {
    // write() is called once per MuJoCo physics step, whereas period can be
    // the controller update period or zero on the intermediate physics step.
    actuator_dt = mj_model_->opt.timestep;
  }
  if (!std::isfinite(actuator_dt) || actuator_dt <= 0.0) {
    actuator_dt = 0.001;
  }

  for (auto& joint : joint_states_) {
    const auto kp = nonnegativeFiniteGain(joint.kp);
    const auto kd = nonnegativeFiniteGain(joint.kd);
    const bool use_shortest_angle_error = isFrontRearDriveJoint(joint.name);
    const double direct_position_error = joint.position_command - joint.position;
    const double position_error =
        use_shortest_angle_error ? angleDifference(joint.position_command, joint.position, 2 * M_PI)
                                 : direct_position_error;
    const double pd_effort = kp * position_error + kd * (joint.velocity_command - joint.velocity);
    auto effort = pd_effort + joint.effort_command;
    if (is_gas_spring_joint(joint)) {
      effort += gas_spring_model_force(joint);
    }
    if (!std::isfinite(effort)) {
      RCLCPP_ERROR(logger_, "Non-finite effort computed for joint '%s'; applying zero effort",
                   joint.name.c_str());
      effort = 0.0;
    }
    if (joint.is_effort_control_enabled && std::isfinite(joint.min_effort_command) &&
        std::isfinite(joint.max_effort_command) &&
        joint.min_effort_command <= joint.max_effort_command) {
      effort = clamp(effort, joint.min_effort_command, joint.max_effort_command);
    }
    joint.actuator_requested_effort = effort;
    effort = apply_joint_actuator_model(joint, effort, actuator_dt);
    if (use_shortest_angle_error) {
      const double raw_position = mj_data_->qpos[joint.mj_pos_adr];
      const double raw_position_deg = raw_position * kRadToDeg;
      const double normalized_position_deg = joint.position * kRadToDeg;
      if ((isNearSigned180Deg(raw_position_deg) || isNearSigned180Deg(normalized_position_deg)) &&
          mj_data_->time - joint.last_wrap_warn_time >= kJointWrapWarnIntervalSec) {
        joint.last_wrap_warn_time = mj_data_->time;
        RCLCPP_WARN(logger_,
                    "MuJoCo front/rear joint near +/-180 deg: joint=%s raw=%.2f deg (%.6f rad), "
                    "position=%.2f deg (%.6f rad), command=%.2f deg (%.6f rad), "
                    "direct_pos_error=%.2f deg (%.6f rad), shortest_pos_error=%.2f deg (%.6f rad), "
                    "velocity=%.6f rad/s, kp=%.3f, kd=%.3f, pd_effort=%.6f, applied_effort=%.6f",
                    joint.name.c_str(), raw_position_deg, raw_position, normalized_position_deg,
                    joint.position, joint.position_command * kRadToDeg, joint.position_command,
                    direct_position_error * kRadToDeg, direct_position_error,
                    position_error * kRadToDeg, position_error, joint.velocity, kp, kd, pd_effort,
                    effort);
      }
    }
    mj_data_->qfrc_applied[joint.mj_vel_adr] = effort;
  }
  return hardware_interface::return_type::OK;
}

bool MujocoSystem::init_sim(mjModel* mujoco_model, mjData* mujoco_data,
                            const urdf::Model& urdf_model,
                            const hardware_interface::HardwareInfo& hardware_info) {
  if (mujoco_model == nullptr || mujoco_data == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("mujoco_system"), "MuJoCo model/data pointer is null");
    return false;
  }
  mj_model_ = mujoco_model;
  mj_data_ = mujoco_data;

  logger_ = rclcpp::get_logger("mujoco_system");

  joint_states_.clear();
  ft_sensor_data_.clear();
  imu_sensor_data_.clear();
  state_interfaces_.clear();
  command_interfaces_.clear();
  last_sim_time_ = -1.0;

  try {
    configure_gas_spring_model(hardware_info);
    if (!register_joints(urdf_model, hardware_info)) {
      return false;
    }
    register_sensors(urdf_model, hardware_info);
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(logger_, "Failed to register MuJoCo hardware interfaces: %s", exception.what());
    return false;
  }

  set_initial_pose();
  return true;
}

bool MujocoSystem::register_joints(const urdf::Model& urdf_model,
                                   const hardware_interface::HardwareInfo& hardware_info) {
  if (hardware_info.joints.empty()) {
    RCLCPP_ERROR(logger_, "ros2_control hardware component declares no joints");
    return false;
  }

  bool all_joints_present = true;
  for (const auto& joint : hardware_info.joints) {
    if (mj_name2id(mj_model_, mjtObj::mjOBJ_JOINT, joint.name.c_str()) < 0) {
      RCLCPP_ERROR_STREAM(logger_,
                          "Failed to find joint in MuJoCo model, joint name: " << joint.name);
      all_joints_present = false;
    }
    if (!urdf_model.getJoint(joint.name)) {
      RCLCPP_ERROR_STREAM(logger_,
                          "Failed to find ros2_control joint in URDF, joint name: " << joint.name);
      all_joints_present = false;
    }
  }
  if (!all_joints_present) {
    RCLCPP_ERROR(logger_,
                 "MuJoCo/URDF joint contract is incomplete; refusing partial initialization");
    return false;
  }

  joint_states_.resize(hardware_info.joints.size());

  for (size_t joint_index = 0; joint_index < hardware_info.joints.size(); joint_index++) {
    auto joint = hardware_info.joints.at(joint_index);
    int mujoco_joint_id = mj_name2id(mj_model_, mjtObj::mjOBJ_JOINT, joint.name.c_str());
    // save information in joint_states_ variable
    JointState joint_state{};
    joint_state.name = joint.name;
    joint_state.mj_joint_type = mj_model_->jnt_type[mujoco_joint_id];
    joint_state.mj_pos_adr = mj_model_->jnt_qposadr[mujoco_joint_id];
    joint_state.mj_vel_adr = mj_model_->jnt_dofadr[mujoco_joint_id];

    joint_states_.at(joint_index) = joint_state;
    JointState& last_joint_state = joint_states_.at(joint_index);
    configure_joint_actuator_model(last_joint_state, joint);

    // get joint limit from urdf
    get_joint_limits(urdf_model.getJoint(last_joint_state.name), last_joint_state.joint_limits);

    // check if mimicked
    if (joint.parameters.find("mimic") != joint.parameters.end()) {
      const auto mimicked_joint = joint.parameters.at("mimic");
      const auto mimicked_joint_it =
          std::find_if(hardware_info.joints.begin(), hardware_info.joints.end(),
                       [&mimicked_joint](const hardware_interface::ComponentInfo& info) {
                         return info.name == mimicked_joint;
                       });
      if (mimicked_joint_it == hardware_info.joints.end()) {
        throw std::runtime_error(std::string("Mimicked joint '") + mimicked_joint + "' not found");
      }
      last_joint_state.is_mimic = true;
      last_joint_state.mimicked_joint_index =
          std::distance(hardware_info.joints.begin(), mimicked_joint_it);

      auto param_it = joint.parameters.find("multiplier");
      if (param_it != joint.parameters.end()) {
        last_joint_state.mimic_multiplier = std::stod(joint.parameters.at("multiplier"));
      } else {
        last_joint_state.mimic_multiplier = 1.0;
      }
    }

    auto get_initial_value = [this](const hardware_interface::InterfaceInfo& interface_info) {
      if (!interface_info.initial_value.empty()) {
        double value = std::stod(interface_info.initial_value);
        return value;
      } else {
        return 0.0;
      }
    };

    // state interfaces
    for (const auto& state_if : joint.state_interfaces) {
      if (state_if.name == hardware_interface::HW_IF_POSITION) {
        state_interfaces_.emplace_back(joint.name, hardware_interface::HW_IF_POSITION,
                                       &last_joint_state.position);
        last_joint_state.position = get_initial_value(state_if);
      } else if (state_if.name == hardware_interface::HW_IF_VELOCITY) {
        state_interfaces_.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY,
                                       &last_joint_state.velocity);
        last_joint_state.velocity = get_initial_value(state_if);
      } else if (state_if.name == hardware_interface::HW_IF_EFFORT) {
        state_interfaces_.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT,
                                       &last_joint_state.effort);
        last_joint_state.effort = get_initial_value(state_if);
      }
    }

    auto get_min_value = [this](const hardware_interface::InterfaceInfo& interface_info) {
      if (!interface_info.min.empty()) {
        double value = std::stod(interface_info.min);
        return value;
      } else {
        return -1 * std::numeric_limits<double>::max();
      }
    };

    auto get_max_value = [this](const hardware_interface::InterfaceInfo& interface_info) {
      if (!interface_info.max.empty()) {
        double value = std::stod(interface_info.max);
        return value;
      } else {
        return std::numeric_limits<double>::max();
      }
    };

    // command interfaces
    // overwrite joint limit with min/max value
    for (const auto& command_if : joint.command_interfaces) {
      if (command_if.name.find(hardware_interface::HW_IF_POSITION) != std::string::npos) {
        command_interfaces_.emplace_back(joint.name, hardware_interface::HW_IF_POSITION,
                                         &last_joint_state.position_command);
        last_joint_state.is_position_control_enabled = true;
        last_joint_state.position_command = last_joint_state.position;
      } else if (command_if.name.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos) {
        command_interfaces_.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY,
                                         &last_joint_state.velocity_command);
        last_joint_state.is_velocity_control_enabled = true;
        last_joint_state.velocity_command = last_joint_state.velocity;
      } else if (command_if.name == hardware_interface::HW_IF_EFFORT) {
        command_interfaces_.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT,
                                         &last_joint_state.effort_command);
        last_joint_state.is_effort_control_enabled = true;
        last_joint_state.effort_command = last_joint_state.effort;
        last_joint_state.min_effort_command = get_min_value(command_if);
        last_joint_state.max_effort_command = get_max_value(command_if);
      } else if (command_if.name == "kp") {
        command_interfaces_.emplace_back(joint.name, "kp", &last_joint_state.kp);
      } else if (command_if.name == "kd") {
        command_interfaces_.emplace_back(joint.name, "kd", &last_joint_state.kd);
      }
      if (command_if.name.find("_pid") != std::string::npos) {
        last_joint_state.is_pid_enabled = true;
      }
    }

    // Get PID gains, if needed
    if (last_joint_state.is_pid_enabled) {
      last_joint_state.position_pid = get_pid_gains(joint, hardware_interface::HW_IF_POSITION);
      last_joint_state.velocity_pid = get_pid_gains(joint, hardware_interface::HW_IF_VELOCITY);
    }
  }
  return true;
}

void MujocoSystem::reset_runtime_state() {
  RCLCPP_INFO(logger_, "MuJoCo time reset detected; clearing cached actuator commands");
  for (auto& joint : joint_states_) {
    const double position = mj_data_->qpos[joint.mj_pos_adr];
    joint.position = std::isfinite(position) ? normalizeAngle(position, 2 * M_PI) : 0.0;
    joint.velocity = 0.0;
    joint.effort = 0.0;
    joint.position_command = joint.position;
    joint.velocity_command = 0.0;
    joint.effort_command = 0.0;
    joint.kp = 0.0;
    joint.kd = 0.0;
    joint.actuator_requested_effort = 0.0;
    joint.actuator_applied_effort = 0.0;
    joint.actuator_applied_effort_rate = 0.0;
    joint.actuator_model_initialized = false;
    joint.actuator_delay_steps = 0;
    joint.actuator_delay_line.clear();
    joint.last_wrap_warn_time = -1.0;
    mj_data_->qfrc_applied[joint.mj_vel_adr] = 0.0;
  }
}

void MujocoSystem::configure_joint_actuator_model(
    JointState& joint_state, const hardware_interface::ComponentInfo& joint_info) {
  auto read_double = [&joint_info](const std::string& name, double fallback) {
    const auto it = joint_info.parameters.find(name);
    if (it == joint_info.parameters.end()) {
      return fallback;
    }
    try {
      const double value = std::stod(it->second);
      return std::isfinite(value) ? value : fallback;
    } catch (const std::exception&) {
      return fallback;
    }
  };

  const auto enabled_it = joint_info.parameters.find("actuator_model_enabled");
  joint_state.actuator_model_enabled =
      enabled_it != joint_info.parameters.end() && parseBool(enabled_it->second);
  joint_state.actuator_time_constant_s =
      nonnegativeFiniteGain(read_double("actuator_time_constant_s", 0.0));
  joint_state.actuator_transport_delay_s =
      nonnegativeFiniteGain(read_double("actuator_transport_delay_s", 0.0));
  joint_state.actuator_dc_gain = nonnegativeFiniteGain(read_double("actuator_dc_gain", 1.0));
  joint_state.actuator_natural_frequency_hz =
      nonnegativeFiniteGain(read_double("actuator_natural_frequency_hz", 0.0));
  joint_state.actuator_damping_ratio =
      nonnegativeFiniteGain(read_double("actuator_damping_ratio", 1.0));
  joint_state.actuator_torque_slew_rate_nm_s =
      nonnegativeFiniteGain(read_double("actuator_torque_slew_rate_nm_s", 0.0));
  joint_state.actuator_output_limit =
      nonnegativeFiniteGain(read_double("actuator_output_limit", 0.0));
  joint_state.actuator_speed_droop_start_rad_s =
      nonnegativeFiniteGain(read_double("actuator_speed_droop_start_rad_s", 0.0));
  joint_state.actuator_speed_droop_per_rad_s =
      nonnegativeFiniteGain(read_double("actuator_speed_droop_per_rad_s", 0.0));
  joint_state.actuator_min_speed_gain =
      clamp(nonnegativeFiniteGain(read_double("actuator_min_speed_gain", 1.0)), 0.0, 1.0);

  if (!joint_state.actuator_model_enabled) {
    return;
  }

  RCLCPP_INFO(logger_,
              "Joint actuator model enabled: joint=%s mode=%s tau=%.3f ms delay=%.3f ms "
              "gain=%.4f natural_frequency=%.2f Hz damping_ratio=%.3f "
              "torque_slew_rate=%.1f Nm/s output_limit=%.1f Nm "
              "speed_droop=[start=%.2f rad/s slope=%.4f/(rad/s) floor=%.2f]",
              joint_state.name.c_str(),
              joint_state.actuator_natural_frequency_hz > 0.0 ? "second_order" : "first_order",
              1000.0 * joint_state.actuator_time_constant_s,
              1000.0 * joint_state.actuator_transport_delay_s, joint_state.actuator_dc_gain,
              joint_state.actuator_natural_frequency_hz, joint_state.actuator_damping_ratio,
              joint_state.actuator_torque_slew_rate_nm_s, joint_state.actuator_output_limit,
              joint_state.actuator_speed_droop_start_rad_s,
              joint_state.actuator_speed_droop_per_rad_s, joint_state.actuator_min_speed_gain);
}

double MujocoSystem::apply_joint_actuator_model(JointState& joint_state, double requested_effort,
                                                double dt) {
  if (!joint_state.actuator_model_enabled || !std::isfinite(requested_effort)) {
    return requested_effort;
  }

  const size_t delay_steps =
      static_cast<size_t>(std::llround(joint_state.actuator_transport_delay_s / dt));
  if (!joint_state.actuator_model_initialized || delay_steps != joint_state.actuator_delay_steps) {
    joint_state.actuator_delay_steps = delay_steps;
    joint_state.actuator_delay_line.assign(delay_steps, requested_effort);
    joint_state.actuator_applied_effort = joint_state.actuator_dc_gain * requested_effort;
    joint_state.actuator_applied_effort_rate = 0.0;
    joint_state.actuator_model_initialized = true;
  }

  double delayed_effort = requested_effort;
  if (joint_state.actuator_delay_steps > 0) {
    delayed_effort = joint_state.actuator_delay_line.front();
    joint_state.actuator_delay_line.pop_front();
    joint_state.actuator_delay_line.push_back(requested_effort);
  }

  const double speed_above_droop_start =
      std::max(0.0, std::abs(joint_state.velocity) - joint_state.actuator_speed_droop_start_rad_s);
  joint_state.actuator_speed_gain =
      clamp(1.0 - joint_state.actuator_speed_droop_per_rad_s * speed_above_droop_start,
            joint_state.actuator_min_speed_gain, 1.0);
  const double target_effort =
      joint_state.actuator_dc_gain * joint_state.actuator_speed_gain * delayed_effort;
  if (joint_state.actuator_natural_frequency_hz > 0.0 && joint_state.actuator_damping_ratio > 0.0) {
    // Continuous second-order closed-loop model:
    //   y'' + 2*zeta*wn*y' + wn^2*y = wn^2*target
    // With a measured torque slew-rate limit, semi-implicit Euler makes the
    // rate saturation explicit and reproduces the nearly linear FOC ramp.
    // Without a slew limit, retain RK4 for the generic second-order model.
    const double wn = 2.0 * M_PI * joint_state.actuator_natural_frequency_hz;
    const double zeta = joint_state.actuator_damping_ratio;
    if (joint_state.actuator_torque_slew_rate_nm_s > 0.0) {
      const double acceleration = wn * wn * (target_effort - joint_state.actuator_applied_effort) -
                                  2.0 * zeta * wn * joint_state.actuator_applied_effort_rate;
      joint_state.actuator_applied_effort_rate = clamp(
          joint_state.actuator_applied_effort_rate + dt * acceleration,
          -joint_state.actuator_torque_slew_rate_nm_s, joint_state.actuator_torque_slew_rate_nm_s);
      joint_state.actuator_applied_effort += dt * joint_state.actuator_applied_effort_rate;
    } else {
      auto derivative = [wn, zeta, target_effort](double effort, double effort_rate) {
        return std::pair<double, double>(
            effort_rate, wn * wn * (target_effort - effort) - 2.0 * zeta * wn * effort_rate);
      };

      const auto k1 =
          derivative(joint_state.actuator_applied_effort, joint_state.actuator_applied_effort_rate);
      const auto k2 = derivative(joint_state.actuator_applied_effort + 0.5 * dt * k1.first,
                                 joint_state.actuator_applied_effort_rate + 0.5 * dt * k1.second);
      const auto k3 = derivative(joint_state.actuator_applied_effort + 0.5 * dt * k2.first,
                                 joint_state.actuator_applied_effort_rate + 0.5 * dt * k2.second);
      const auto k4 = derivative(joint_state.actuator_applied_effort + dt * k3.first,
                                 joint_state.actuator_applied_effort_rate + dt * k3.second);

      joint_state.actuator_applied_effort +=
          dt * (k1.first + 2.0 * k2.first + 2.0 * k3.first + k4.first) / 6.0;
      joint_state.actuator_applied_effort_rate +=
          dt * (k1.second + 2.0 * k2.second + 2.0 * k3.second + k4.second) / 6.0;
    }
  } else if (joint_state.actuator_time_constant_s <= 1.0e-6) {
    joint_state.actuator_applied_effort = target_effort;
    joint_state.actuator_applied_effort_rate = 0.0;
  } else {
    const double alpha = 1.0 - std::exp(-dt / joint_state.actuator_time_constant_s);
    joint_state.actuator_applied_effort +=
        alpha * (target_effort - joint_state.actuator_applied_effort);
  }

  if (joint_state.actuator_output_limit > 0.0) {
    const double unclipped_effort = joint_state.actuator_applied_effort;
    joint_state.actuator_applied_effort = clamp(
        unclipped_effort, -joint_state.actuator_output_limit, joint_state.actuator_output_limit);
    if (joint_state.actuator_applied_effort != unclipped_effort &&
        joint_state.actuator_applied_effort * joint_state.actuator_applied_effort_rate > 0.0) {
      joint_state.actuator_applied_effort_rate = 0.0;
    }
  }
  return joint_state.actuator_applied_effort;
}

void MujocoSystem::register_sensors(const urdf::Model& /* urdf_model */,
                                    const hardware_interface::HardwareInfo& hardware_info) {
  // Assuming force/torque sensor end with "_fts" in the name,
  // and IMU sensor end with "_imu" in the name
  for (size_t sensor_index = 0; sensor_index < hardware_info.sensors.size(); sensor_index++) {
    auto sensor = hardware_info.sensors.at(sensor_index);
    std::string sensor_name = sensor.name;
    sensor_name = sensor_name.substr(0, sensor_name.rfind('_'));

    if (sensor.name.find("fts") != std::string::npos) {
      FTSensorData sensor_data;
      sensor_data.name = sensor_name;
      sensor_data.force.name = sensor_name + "_force";
      sensor_data.torque.name = sensor_name + "_torque";

      int force_sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, sensor_data.force.name.c_str());
      int torque_sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, sensor_data.torque.name.c_str());

      if (force_sensor_id == -1 || torque_sensor_id == -1) {
        RCLCPP_ERROR_STREAM(
            logger_,
            "Failed to find force/torque sensor in mujoco model, sensor name: " << sensor.name);
        continue;
      }

      sensor_data.force.mj_sensor_index = mj_model_->sensor_adr[force_sensor_id];
      sensor_data.torque.mj_sensor_index = mj_model_->sensor_adr[torque_sensor_id];

      ft_sensor_data_.push_back(sensor_data);
      auto& last_sensor_data = ft_sensor_data_.back();

      for (const auto& state_if : sensor.state_interfaces) {
        if (state_if.name == "force.x") {
          state_interfaces_.emplace_back(sensor.name, state_if.name,
                                         &last_sensor_data.force.data.x());
        } else if (state_if.name == "force.y") {
          state_interfaces_.emplace_back(sensor.name, state_if.name,
                                         &last_sensor_data.force.data.y());
        } else if (state_if.name == "force.z") {
          state_interfaces_.emplace_back(sensor.name, state_if.name,
                                         &last_sensor_data.force.data.z());
        } else if (state_if.name == "torque.x") {
          state_interfaces_.emplace_back(sensor.name, state_if.name,
                                         &last_sensor_data.torque.data.x());
        } else if (state_if.name == "torque.y") {
          state_interfaces_.emplace_back(sensor.name, state_if.name,
                                         &last_sensor_data.torque.data.y());
        } else if (state_if.name == "torque.z") {
          state_interfaces_.emplace_back(sensor.name, state_if.name,
                                         &last_sensor_data.torque.data.z());
        }
      }
    } else if (sensor.name.find("imu") != std::string::npos) {
      IMUSensorData sensor_data;
      sensor_data.name = sensor_name;
      // MJCF: 四元数 / 陀螺 / 加速度计为三个独立命名的 sensor（见 robot_descriptions/*/mjcf）
      const char* k_frame_quat = "imu";
      const char* k_gyro = "gyro";
      const char* k_accel = "accelerometer";
      sensor_data.orientation.name = k_frame_quat;
      sensor_data.angular_velocity.name = k_gyro;
      sensor_data.linear_acceleration.name = k_accel;

      int quat_id = mj_name2id(mj_model_, mjOBJ_SENSOR, sensor_data.orientation.name.c_str());
      int gyro_id = mj_name2id(mj_model_, mjOBJ_SENSOR, sensor_data.angular_velocity.name.c_str());
      int accel_id =
          mj_name2id(mj_model_, mjOBJ_SENSOR, sensor_data.linear_acceleration.name.c_str());

      if (quat_id == -1 || gyro_id == -1 || accel_id == -1) {
        RCLCPP_ERROR_STREAM(
            logger_, "Failed to find IMU sensor in mujoco model, sensor name: " << sensor.name);
        continue;
      }

      sensor_data.orientation.mj_sensor_index = mj_model_->sensor_adr[quat_id];
      sensor_data.angular_velocity.mj_sensor_index = mj_model_->sensor_adr[gyro_id];
      sensor_data.linear_acceleration.mj_sensor_index = mj_model_->sensor_adr[accel_id];

      imu_sensor_data_.push_back(sensor_data);
      auto& last_sensor_data = imu_sensor_data_.back();

      for (const auto& state_if : sensor.state_interfaces) {
        if (state_if.name == "orientation.x") {
          state_interfaces_.emplace_back(sensor.name, state_if.name,
                                         &last_sensor_data.orientation.data.x());
        } else if (state_if.name == "orientation.y") {
          state_interfaces_.emplace_back(sensor.name, state_if.name,
                                         &last_sensor_data.orientation.data.y());
        } else if (state_if.name == "orientation.z") {
          state_interfaces_.emplace_back(sensor.name, state_if.name,
                                         &last_sensor_data.orientation.data.z());
        } else if (state_if.name == "orientation.w") {
          state_interfaces_.emplace_back(sensor.name, state_if.name,
                                         &last_sensor_data.orientation.data.w());
        } else if (state_if.name == "angular_velocity.x") {
          state_interfaces_.emplace_back(sensor.name, state_if.name,
                                         &last_sensor_data.angular_velocity.data.x());
        } else if (state_if.name == "angular_velocity.y") {
          state_interfaces_.emplace_back(sensor.name, state_if.name,
                                         &last_sensor_data.angular_velocity.data.y());
        } else if (state_if.name == "angular_velocity.z") {
          state_interfaces_.emplace_back(sensor.name, state_if.name,
                                         &last_sensor_data.angular_velocity.data.z());
        } else if (state_if.name == "linear_acceleration.x") {
          state_interfaces_.emplace_back(sensor.name, state_if.name,
                                         &last_sensor_data.linear_acceleration.data.x());
        } else if (state_if.name == "linear_acceleration.y") {
          state_interfaces_.emplace_back(sensor.name, state_if.name,
                                         &last_sensor_data.linear_acceleration.data.y());
        } else if (state_if.name == "linear_acceleration.z") {
          state_interfaces_.emplace_back(sensor.name, state_if.name,
                                         &last_sensor_data.linear_acceleration.data.z());
        }
      }
    }
  }
}

void MujocoSystem::configure_gas_spring_model(
    const hardware_interface::HardwareInfo& hardware_info) {
  auto read_double = [&hardware_info](const std::string& name, double fallback) {
    const auto it = hardware_info.hardware_parameters.find(name);
    if (it == hardware_info.hardware_parameters.end()) {
      return fallback;
    }
    try {
      const double value = std::stod(it->second);
      return std::isfinite(value) ? value : fallback;
    } catch (const std::exception&) {
      return fallback;
    }
  };

  auto read_bool = [&hardware_info](const std::string& name, bool fallback) {
    const auto it = hardware_info.hardware_parameters.find(name);
    if (it == hardware_info.hardware_parameters.end()) {
      return fallback;
    }
    return parseBool(it->second);
  };

  gas_spring_model_.enabled = read_bool("gas_spring_model_enabled", gas_spring_model_.enabled);
  gas_spring_model_.q_min = read_double("gas_spring_q_min", gas_spring_model_.q_min);
  gas_spring_model_.q_max = read_double("gas_spring_q_max", gas_spring_model_.q_max);
  gas_spring_model_.force_at_q_min =
      read_double("gas_spring_force_at_q_min", gas_spring_model_.force_at_q_min);
  gas_spring_model_.force_at_q_max =
      read_double("gas_spring_force_at_q_max", gas_spring_model_.force_at_q_max);
  gas_spring_model_.manual_passive_enabled =
      read_bool("gas_spring_manual_passive_enabled", gas_spring_model_.manual_passive_enabled);
  gas_spring_model_.manual_viscous_damping =
      read_double("gas_spring_manual_viscous_damping", gas_spring_model_.manual_viscous_damping);
  gas_spring_model_.manual_static_friction =
      read_double("gas_spring_manual_static_friction", gas_spring_model_.manual_static_friction);
  gas_spring_model_.manual_velocity_epsilon =
      read_double("gas_spring_manual_velocity_epsilon", gas_spring_model_.manual_velocity_epsilon);

  if (gas_spring_model_.q_max <= gas_spring_model_.q_min) {
    RCLCPP_WARN(logger_, "Invalid gas spring q range [%.6f, %.6f], disabling gas spring model.",
                gas_spring_model_.q_min, gas_spring_model_.q_max);
    gas_spring_model_.enabled = false;
  }
  gas_spring_model_.manual_viscous_damping =
      nonnegativeFiniteGain(gas_spring_model_.manual_viscous_damping);
  gas_spring_model_.manual_static_friction =
      nonnegativeFiniteGain(gas_spring_model_.manual_static_friction);
  gas_spring_model_.manual_velocity_epsilon =
      std::max(1.0e-6, nonnegativeFiniteGain(gas_spring_model_.manual_velocity_epsilon));

  RCLCPP_INFO(logger_,
              "Gas spring model %s: q=[%.4f, %.4f] m force=[%.1f, %.1f] N; manual passive %s "
              "damping=%.2f N/(m/s) friction=%.2f N",
              gas_spring_model_.enabled ? "enabled" : "disabled", gas_spring_model_.q_min,
              gas_spring_model_.q_max, gas_spring_model_.force_at_q_min,
              gas_spring_model_.force_at_q_max,
              gas_spring_model_.manual_passive_enabled ? "enabled" : "disabled",
              gas_spring_model_.manual_viscous_damping, gas_spring_model_.manual_static_friction);
}

double MujocoSystem::gas_spring_model_force(const JointState& joint) const {
  if (!gas_spring_model_.enabled) {
    return 0.0;
  }

  const double q = clamp(joint.position, gas_spring_model_.q_min, gas_spring_model_.q_max);
  const double span = gas_spring_model_.q_max - gas_spring_model_.q_min;
  const double ratio = (q - gas_spring_model_.q_min) / span;
  const double spring_force =
      gas_spring_model_.force_at_q_min +
      (gas_spring_model_.force_at_q_max - gas_spring_model_.force_at_q_min) * ratio;
  double manual_passive_force = 0.0;
  if (gas_spring_model_.manual_passive_enabled) {
    manual_passive_force =
        -gas_spring_model_.manual_viscous_damping * joint.velocity -
        gas_spring_model_.manual_static_friction *
            std::tanh(joint.velocity / gas_spring_model_.manual_velocity_epsilon);
  }

  return spring_force + manual_passive_force;
}

bool MujocoSystem::is_gas_spring_joint(const JointState& joint) const {
  return joint.name == "left_spring2_joint" || joint.name == "right_spring2_joint";
}

void MujocoSystem::set_initial_pose() {
  for (auto& joint_state : joint_states_) {
    mj_data_->qpos[joint_state.mj_pos_adr] = joint_state.position;
  }
}

void MujocoSystem::get_joint_limits(urdf::JointConstSharedPtr urdf_joint,
                                    joint_limits::JointLimits& joint_limits) {
  if (urdf_joint->limits) {
    joint_limits.min_position = urdf_joint->limits->lower;
    joint_limits.max_position = urdf_joint->limits->upper;
    joint_limits.max_velocity = urdf_joint->limits->velocity;
    joint_limits.max_effort = urdf_joint->limits->effort;
  }
}

control_toolbox::Pid MujocoSystem::get_pid_gains(
    const hardware_interface::ComponentInfo& joint_info, std::string command_interface) {
  double kp, ki, kd, i_max, i_min;
  std::string key;
  key = command_interface + std::string(PARAM_KP);
  if (joint_info.parameters.find(key) != joint_info.parameters.end()) {
    kp = std::stod(joint_info.parameters.at(key));
  } else {
    kp = 0.0;
  }

  key = command_interface + std::string(PARAM_KI);
  if (joint_info.parameters.find(key) != joint_info.parameters.end()) {
    ki = std::stod(joint_info.parameters.at(key));
  } else {
    ki = 0.0;
  }

  key = command_interface + std::string(PARAM_KD);
  if (joint_info.parameters.find(key) != joint_info.parameters.end()) {
    kd = std::stod(joint_info.parameters.at(key));
  } else {
    kd = 0.0;
  }

  bool enable_anti_windup = false;
  key = command_interface + std::string(PARAM_I_MAX);
  if (joint_info.parameters.find(key) != joint_info.parameters.end()) {
    i_max = std::stod(joint_info.parameters.at(key));
    enable_anti_windup = true;
  } else {
    i_max = std::numeric_limits<double>::max();
  }

  key = command_interface + std::string(PARAM_I_MIN);
  if (joint_info.parameters.find(key) != joint_info.parameters.end()) {
    i_min = std::stod(joint_info.parameters.at(key));
    enable_anti_windup = true;
  } else {
    i_min = std::numeric_limits<double>::lowest();
  }

  return control_toolbox::Pid(kp, ki, kd, i_max, i_min, enable_anti_windup);
}
}  // namespace mujoco_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control::MujocoSystem,
                       mujoco_ros2_control::MujocoSystemInterface)
