// Copyright (c) 2025 SCUTRobotLab
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "template_ros2_controller/template_ros2_controller.hpp"

#include <hardware_interface/version.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/qos.hpp>

namespace robot_locomotion {
namespace {

double finiteOrZero(double value) { return std::isfinite(value) ? value : 0.0; }

double nonnegativeFinite(double value) {
  return std::isfinite(value) && value >= 0.0 ? value : 0.0;
}

std::uint64_t steadyNowNanoseconds() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::steady_clock::now().time_since_epoch())
                                        .count());
}

double positionError(const std::string& joint_name, double position_command, double position) {
  const double direct_error = position_command - position;
  const bool revolute_leg_joint =
      joint_name == "left_front1_joint" || joint_name == "left_rear1_joint" ||
      joint_name == "right_front1_joint" || joint_name == "right_rear1_joint";
  constexpr double kTwoPi = 6.28318530717958647692;
  return revolute_leg_joint ? std::remainder(direct_error, kTwoPi) : direct_error;
}

bool finiteArray(const std::array<double, 3>& values) {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

bool finiteArray(const std::array<double, 4>& values) {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

}  // namespace

bool validateControllerParameters(const template_ros2_controller::Params& params,
                                  std::string& error) {
  struct ArraySpec {
    const char* name;
    const std::vector<double>* values;
    size_t expected_size;
    bool nonnegative;
  };
  const std::array<ArraySpec, 33> arrays{{
      {"joint_stiffness", &params.joint_stiffness, 8, true},
      {"joint_damping", &params.joint_damping, 8, true},
      {"joint_action_scale", &params.joint_action_scale, 8, false},
      {"joint_output_max", &params.joint_output_max, 8, false},
      {"joint_output_min", &params.joint_output_min, 8, false},
      {"joint_bias", &params.joint_bias, 8, false},
      {"default_dof_pos", &params.default_dof_pos, 8, false},
      {"prepare_dof_pos", &params.prepare_dof_pos, 4, false},
      {"prepare_kp", &params.prepare_kp, 4, true},
      {"prepare_kd", &params.prepare_kd, 4, true},
      {"imu_gyro_noise_stddev", &params.imu_gyro_noise_stddev, 3, true},
      {"imu_accel_noise_stddev", &params.imu_accel_noise_stddev, 3, true},
      {"policy_input_cmd_scale", &params.policy_input_cmd_scale, 4, false},
      {"policy_input_cmd_max", &params.policy_input_cmd_max, 4, false},
      {"policy_input_cmd_min", &params.policy_input_cmd_min, 4, false},
      {"policy_input_ang_vel_scale", &params.policy_input_ang_vel_scale, 3, false},
      {"policy_input_ang_vel_max", &params.policy_input_ang_vel_max, 3, false},
      {"policy_input_ang_vel_min", &params.policy_input_ang_vel_min, 3, false},
      {"policy_input_gravity_scale", &params.policy_input_gravity_scale, 3, false},
      {"policy_input_gravity_max", &params.policy_input_gravity_max, 3, false},
      {"policy_input_gravity_min", &params.policy_input_gravity_min, 3, false},
      {"policy_input_joint_pos_scale", &params.policy_input_joint_pos_scale, 6, false},
      {"policy_input_joint_pos_max", &params.policy_input_joint_pos_max, 6, false},
      {"policy_input_joint_pos_min", &params.policy_input_joint_pos_min, 6, false},
      {"policy_input_joint_vel_scale", &params.policy_input_joint_vel_scale, 4, false},
      {"policy_input_joint_vel_max", &params.policy_input_joint_vel_max, 4, false},
      {"policy_input_joint_vel_min", &params.policy_input_joint_vel_min, 4, false},
      {"policy_input_wheel_vel_scale", &params.policy_input_wheel_vel_scale, 2, false},
      {"policy_input_wheel_vel_max", &params.policy_input_wheel_vel_max, 2, false},
      {"policy_input_wheel_vel_min", &params.policy_input_wheel_vel_min, 2, false},
      {"policy_input_action_scale", &params.policy_input_action_scale, 6, false},
      {"policy_input_action_max", &params.policy_input_action_max, 6, false},
      {"policy_input_action_min", &params.policy_input_action_min, 6, false},
  }};

  for (const auto& spec : arrays) {
    if (spec.values->size() != spec.expected_size) {
      std::ostringstream message;
      message << spec.name << " must contain exactly " << spec.expected_size << " values; got "
              << spec.values->size();
      error = message.str();
      return false;
    }
    for (size_t index = 0; index < spec.values->size(); ++index) {
      const double value = spec.values->at(index);
      if (!std::isfinite(value)) {
        error = std::string(spec.name) + "[" + std::to_string(index) + "] must be finite";
        return false;
      }
      if (spec.nonnegative && value < 0.0) {
        error = std::string(spec.name) + "[" + std::to_string(index) + "] must be nonnegative";
        return false;
      }
    }
  }

  const std::array<std::pair<const char*, double>, 12> scalars{{
      {"rl_action_filter_alpha", params.rl_action_filter_alpha},
      {"motion_command_timeout_sec", params.motion_command_timeout_sec},
      {"motion_linear_x_min", params.motion_linear_x_min},
      {"motion_linear_x_max", params.motion_linear_x_max},
      {"motion_angular_z_min", params.motion_angular_z_min},
      {"motion_angular_z_max", params.motion_angular_z_max},
      {"default_command_height", params.default_command_height},
      {"command_height_min", params.command_height_min},
      {"command_height_max", params.command_height_max},
      {"prepare_max_velocity", params.prepare_max_velocity},
      {"joint_position_noise_stddev", params.joint_position_noise_stddev},
      {"joint_velocity_noise_stddev", params.joint_velocity_noise_stddev},
  }};
  for (const auto& [name, value] : scalars) {
    if (!std::isfinite(value)) {
      error = std::string(name) + " must be finite";
      return false;
    }
  }
  if (params.rl_action_filter_alpha < 0.0 || params.rl_action_filter_alpha > 1.0) {
    error = "rl_action_filter_alpha must be in [0, 1]";
    return false;
  }
  if (params.motion_command_timeout_sec <= 0.0) {
    error = "motion_command_timeout_sec must be positive";
    return false;
  }
  if (params.prepare_max_velocity <= 0.0) {
    error = "prepare_max_velocity must be positive";
    return false;
  }
  if (params.joint_position_noise_stddev < 0.0 || params.joint_velocity_noise_stddev < 0.0) {
    error = "joint noise standard deviations must be nonnegative";
    return false;
  }
  if (params.motion_linear_x_min > params.motion_linear_x_max) {
    error = "motion_linear_x_min must be <= motion_linear_x_max";
    return false;
  }
  if (params.motion_angular_z_min > params.motion_angular_z_max) {
    error = "motion_angular_z_min must be <= motion_angular_z_max";
    return false;
  }
  if (params.command_height_min > params.command_height_max) {
    error = "command_height_min must be <= command_height_max";
    return false;
  }
  if (params.default_command_height < params.command_height_min ||
      params.default_command_height > params.command_height_max) {
    error = "default_command_height must be within [command_height_min, command_height_max]";
    return false;
  }

  struct BoundsSpec {
    const char* name;
    const std::vector<double>* minimum;
    const std::vector<double>* maximum;
  };
  const std::array<BoundsSpec, 8> bounds{{
      {"joint_output", &params.joint_output_min, &params.joint_output_max},
      {"policy_input_cmd", &params.policy_input_cmd_min, &params.policy_input_cmd_max},
      {"policy_input_ang_vel", &params.policy_input_ang_vel_min, &params.policy_input_ang_vel_max},
      {"policy_input_gravity", &params.policy_input_gravity_min, &params.policy_input_gravity_max},
      {"policy_input_joint_pos", &params.policy_input_joint_pos_min,
       &params.policy_input_joint_pos_max},
      {"policy_input_joint_vel", &params.policy_input_joint_vel_min,
       &params.policy_input_joint_vel_max},
      {"policy_input_wheel_vel", &params.policy_input_wheel_vel_min,
       &params.policy_input_wheel_vel_max},
      {"policy_input_action", &params.policy_input_action_min, &params.policy_input_action_max},
  }};
  for (const auto& spec : bounds) {
    for (size_t index = 0; index < spec.minimum->size(); ++index) {
      if (spec.minimum->at(index) > spec.maximum->at(index)) {
        error = std::string(spec.name) + "_min[" + std::to_string(index) +
                "] must be <= the corresponding maximum";
        return false;
      }
    }
  }

  if (params.lowlevel_output_mode != "torque" && params.lowlevel_output_mode != "hardware_pd" &&
      params.lowlevel_output_mode != "hardware_pd_vel") {
    error = "lowlevel_output_mode is not supported";
    return false;
  }
  if (params.rl_action_filter_type != "none" && params.rl_action_filter_type != "moving_avg" &&
      params.rl_action_filter_type != "lowpass") {
    error = "rl_action_filter_type is not supported";
    return false;
  }

  error.clear();
  return true;
}

bool validateJointContract(const std::vector<std::string>& joint_names, std::string& error) {
  static constexpr std::array<const char*, 8> kExpectedJointNames{
      "left_front1_joint", "left_rear1_joint",  "right_front1_joint", "right_rear1_joint",
      "left_wheel_joint",  "right_wheel_joint", "left_spring2_joint", "right_spring2_joint",
  };
  if (joint_names.size() != kExpectedJointNames.size()) {
    error = "joints must contain exactly 8 names in the public contract order; got " +
            std::to_string(joint_names.size());
    return false;
  }
  for (size_t index = 0; index < kExpectedJointNames.size(); ++index) {
    if (joint_names[index] != kExpectedJointNames[index]) {
      error = "joints[" + std::to_string(index) + "] must be '" + kExpectedJointNames[index] +
              "'; got '" + joint_names[index] + "'";
      return false;
    }
  }
  error.clear();
  return true;
}

geometry_msgs::msg::Twist limitMotionCommand(const geometry_msgs::msg::Twist& command,
                                             const template_ros2_controller::Params& params) {
  geometry_msgs::msg::Twist limited;
  limited.linear.x =
      std::clamp(command.linear.x, params.motion_linear_x_min, params.motion_linear_x_max);
  limited.angular.z =
      std::clamp(command.angular.z, params.motion_angular_z_min, params.motion_angular_z_max);
  return limited;
}

double limitHeightCommand(double height, const template_ros2_controller::Params& params) {
  return std::clamp(height, params.command_height_min, params.command_height_max);
}

controller_interface::CallbackReturn TemplateRos2Controller::on_init() {
  try {
    bool use_prefixed_params = false;
    const auto declare_string_array = [this, &use_prefixed_params](
                                          const std::string& name,
                                          const std::vector<std::string>& default_value) {
      auto value = auto_declare<std::vector<std::string>>(name, default_value);
      if (!value.empty()) {
        return value;
      }
      const std::string prefixed_name = "template_ros2_controller." + name;
      auto prefixed_value = auto_declare<std::vector<std::string>>(prefixed_name, default_value);
      if (!prefixed_value.empty()) {
        use_prefixed_params = true;
      }
      return prefixed_value;
    };

    joint_names_ = declare_string_array("joints", {});
    command_interface_types_ = declare_string_array("command_interfaces", {});
    state_interface_types_ = declare_string_array("state_interfaces", {});
    sensor_names_ = declare_string_array("sensors", {});
    controller_param_prefix_ = use_prefixed_params ? "template_ros2_controller" : "";

    if (sensor_names_.size() != 1) {
      RCLCPP_ERROR(get_node()->get_logger(), "Exactly one IMU sensor must be configured");
      return controller_interface::CallbackReturn::ERROR;
    }
    imu_sensor_ = std::make_unique<semantic_components::IMUSensor>(sensor_names_.front());

    param_listener_ = std::make_shared<template_ros2_controller::ParamListener>(
        get_node(), controller_param_prefix_);
    params_ = param_listener_->get_params();
    std::string validation_error;
    if (!validateControllerParameters(params_, validation_error)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Invalid controller configuration: %s",
                   validation_error.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    motion_command_timeout_sec_ = params_.motion_command_timeout_sec;
    latest_height_ = params_.default_command_height;
    last_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    state_machine_ = std::make_unique<StateMachine>(get_node()->get_logger(), get_node());
    configureStateMachine();

    std::filesystem::path policy_path(params_.rl_model_path);
    if (policy_path.is_relative()) {
      policy_path = std::filesystem::path(
                        ament_index_cpp::get_package_share_directory("template_ros2_controller")) /
                    policy_path;
    }
    rl_inference_ready_ =
        state_machine_->initializeRLInference(policy_path.string(), params_.rl_inference_frequency);
    if (!rl_inference_ready_) {
      RCLCPP_ERROR(get_node()->get_logger(), "Unable to initialize the bundled ONNX policy");
      return controller_interface::CallbackReturn::ERROR;
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Controller initialization failed: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

void TemplateRos2Controller::configureStateMachine() {
  state_machine_->setInferenceFrequency(params_.rl_inference_frequency);
  state_machine_->setPrintInferenceTime(params_.rl_print_inference_time);
  state_machine_->setPublishNetworkIO(params_.rl_publish_network_io);
  state_machine_->setLowlevelOutputMode(params_.lowlevel_output_mode);
  state_machine_->setActionFilterConfig(params_.rl_action_filter_type,
                                        params_.rl_action_filter_window,
                                        params_.rl_action_filter_alpha);
  state_machine_->setJointParams(params_.joint_stiffness, params_.joint_damping,
                                 params_.joint_action_scale, params_.joint_output_max,
                                 params_.joint_output_min, params_.joint_bias,
                                 params_.default_dof_pos);
  state_machine_->setPrepareParams(params_.prepare_dof_pos, params_.prepare_kp, params_.prepare_kd,
                                   params_.prepare_max_velocity);
  state_machine_->setObservationDelaySteps(params_.observation_delay_steps);
  state_machine_->setObservationDelayEnabled(params_.enable_delay);
  state_machine_->setNoiseEnabled(params_.enable_noise);
  state_machine_->setNoiseParams(params_.imu_gyro_noise_stddev, params_.imu_accel_noise_stddev,
                                 params_.joint_position_noise_stddev,
                                 params_.joint_velocity_noise_stddev);
  state_machine_->setPolicyInputParams(
      params_.policy_input_cmd_scale, params_.policy_input_cmd_max, params_.policy_input_cmd_min,
      params_.policy_input_ang_vel_scale, params_.policy_input_ang_vel_max,
      params_.policy_input_ang_vel_min, params_.policy_input_gravity_scale,
      params_.policy_input_gravity_max, params_.policy_input_gravity_min,
      params_.policy_input_joint_pos_scale, params_.policy_input_joint_pos_max,
      params_.policy_input_joint_pos_min, params_.policy_input_joint_vel_scale,
      params_.policy_input_joint_vel_max, params_.policy_input_joint_vel_min,
      params_.policy_input_wheel_vel_scale, params_.policy_input_wheel_vel_max,
      params_.policy_input_wheel_vel_min, params_.policy_input_action_scale,
      params_.policy_input_action_max, params_.policy_input_action_min);
}

controller_interface::CallbackReturn TemplateRos2Controller::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  std::string validation_error;
  if (!validateControllerParameters(params_, validation_error)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid controller configuration: %s",
                 validation_error.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!validateJointContract(joint_names_, validation_error)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid joint contract: %s", validation_error.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  joints_.clear();
  joints_.reserve(joint_names_.size());
  for (const auto& name : joint_names_) {
    auto joint = std::make_shared<Joint>();
    joint->name = name;
    joints_.push_back(std::move(joint));
  }
  robot_state_ = RobotState{};
  robot_state_.joints.resize(joint_names_.size());
  for (size_t index = 0; index < joint_names_.size(); ++index) {
    robot_state_.joints[index].name = joint_names_[index];
  }
  joint_command_snapshots_.assign(joint_names_.size(), JointCommandSnapshot{});

  {
    std::lock_guard<std::mutex> lock(motion_cmd_mutex_);
    latest_lin_vel_x_ = 0.0;
    latest_ang_vel_z_ = 0.0;
    latest_height_ = params_.default_command_height;
    motion_cmd_received_ = false;
    height_cmd_received_ = false;
    last_motion_command_receive_ns_ = 0;
    last_height_command_receive_ns_ = 0;
  }

  auto command_qos = rclcpp::SystemDefaultsQoS();
  command_qos.keep_last(1).best_effort();
  motion_cmd_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
      "motion_command", command_qos,
      std::bind(&TemplateRos2Controller::motionCommandCallback, this, std::placeholders::_1));
  height_cmd_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64>(
      "height_command", command_qos,
      std::bind(&TemplateRos2Controller::heightCommandCallback, this, std::placeholders::_1));
  state_cmd_subscriber_ = get_node()->create_subscription<std_msgs::msg::Int32>(
      "state_command", command_qos,
      std::bind(&TemplateRos2Controller::stateCommandCallback, this, std::placeholders::_1));

  current_state_publisher_ =
      get_node()->create_publisher<std_msgs::msg::Int32>("current_state", 10);
  joint_command_publisher_ =
      get_node()->create_publisher<sensor_msgs::msg::JointState>("joint_commands", 10);
  joint_final_torque_publisher_ =
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("joint_final_torque", 10);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
TemplateRos2Controller::command_interface_configuration() const {
  std::vector<std::string> names;
  for (const auto& joint : joints_) {
    for (const auto& interface_type : command_interface_types_) {
      names.push_back(joint->name + "/" + interface_type);
    }
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
}

controller_interface::InterfaceConfiguration TemplateRos2Controller::state_interface_configuration()
    const {
  std::vector<std::string> names;
  for (const auto& joint : joints_) {
    for (const auto& interface_type : state_interface_types_) {
      names.push_back(joint->name + "/" + interface_type);
    }
  }
  if (imu_sensor_) {
    const auto imu_interfaces = imu_sensor_->get_state_interface_names();
    names.insert(names.end(), imu_interfaces.begin(), imu_interfaces.end());
  }
  if (params_.use_dt7) {
    names.emplace_back("dt7/cmd_state");
    names.emplace_back("dt7/cmd_vel_x");
    names.emplace_back("dt7/cmd_omega_z");
    names.emplace_back("dt7/cmd_height");
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
}

controller_interface::CallbackReturn TemplateRos2Controller::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  const auto bind_command = [this](const std::shared_ptr<Joint>& joint, const std::string& name,
                                   LoanedCommandInterface& destination) {
    const auto iterator = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                                       [&joint, &name](const auto& interface) {
                                         return interface.get_prefix_name() == joint->name &&
                                                interface.get_interface_name() == name;
                                       });
    if (iterator == command_interfaces_.end()) {
      return false;
    }
    destination = std::ref(*iterator);
    return true;
  };
  const auto bind_state = [this](const std::shared_ptr<Joint>& joint, const std::string& name,
                                 LoanedStateInterface& destination) {
    const auto iterator = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
                                       [&joint, &name](const auto& interface) {
                                         return interface.get_prefix_name() == joint->name &&
                                                interface.get_interface_name() == name;
                                       });
    if (iterator == state_interfaces_.end()) {
      return false;
    }
    destination = std::ref(*iterator);
    return true;
  };

  for (const auto& joint : joints_) {
    const bool complete =
        bind_command(joint, hardware_interface::HW_IF_POSITION, joint->position_command_handle) &&
        bind_command(joint, hardware_interface::HW_IF_VELOCITY, joint->velocity_command_handle) &&
        bind_command(joint, hardware_interface::HW_IF_EFFORT, joint->effort_command_handle) &&
        bind_command(joint, "kp", joint->kp_command_handle) &&
        bind_command(joint, "kd", joint->kd_command_handle) &&
        bind_state(joint, hardware_interface::HW_IF_POSITION, joint->position_handle) &&
        bind_state(joint, hardware_interface::HW_IF_VELOCITY, joint->velocity_handle) &&
        bind_state(joint, hardware_interface::HW_IF_EFFORT, joint->effort_handle);
    if (!complete) {
      RCLCPP_ERROR(get_node()->get_logger(), "Incomplete ros2_control interface set for %s",
                   joint->name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }
  }

  if (params_.use_dt7) {
    const auto bind_dt7 = [this](const std::string& name, LoanedStateInterface& destination) {
      const auto iterator = std::find_if(
          state_interfaces_.begin(), state_interfaces_.end(), [&name](const auto& interface) {
            return interface.get_prefix_name() == "dt7" && interface.get_interface_name() == name;
          });
      if (iterator == state_interfaces_.end()) {
        return false;
      }
      destination = std::ref(*iterator);
      return true;
    };
    const bool complete = bind_dt7("cmd_state", dt7_cmd_state_handle_) &&
                          bind_dt7("cmd_vel_x", dt7_cmd_vel_x_handle_) &&
                          bind_dt7("cmd_omega_z", dt7_cmd_omega_z_handle_) &&
                          bind_dt7("cmd_height", dt7_cmd_height_handle_);
    if (!complete) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "use_dt7 requires all four ordinary dt7 state interfaces");
      return controller_interface::CallbackReturn::FAILURE;
    }
    dt7_previous_state_ = -1;
    RCLCPP_INFO(get_node()->get_logger(),
                "Ordinary DT7 input enabled (state, forward speed, yaw rate, height; no jump)");
  }

  imu_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  resetControllerRuntime(get_node()->now());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TemplateRos2Controller::update(const rclcpp::Time& time,
                                                                 const rclcpp::Duration& period) {
  if (last_update_time_.nanoseconds() > 0 && time < last_update_time_) {
    RCLCPP_WARN(get_node()->get_logger(),
                "ROS time moved backwards; resetting controller and policy runtime");
    resetControllerRuntime(time);
  }
  last_update_time_ = time;

  if (!updateRobotState(time, period)) {
    writeSafeJointCommands();
    return controller_interface::return_type::ERROR;
  }
  if (auto_enter_rl_pending_ && rl_inference_ready_ &&
      state_machine_->getCurrentState() != ControllerState::INIT) {
    state_machine_->setTargetState(ControllerState::RL);
    auto_enter_rl_pending_ = false;
  }
  state_machine_->update(robot_state_, time, period);
  const ControllerState state = state_machine_->getCurrentState();
  if (state == ControllerState::RL && state_machine_->getTargetState() == ControllerState::IDLE) {
    writeSafeJointCommands();
  } else {
    writeControllerCommands(state);
  }

  if (current_state_publisher_) {
    std_msgs::msg::Int32 message;
    message.data = static_cast<int32_t>(state);
    current_state_publisher_->publish(message);
  }
  return controller_interface::return_type::OK;
}

bool TemplateRos2Controller::updateRobotState(const rclcpp::Time& time,
                                              const rclcpp::Duration& period) {
  const auto read_value = [this](const LoanedStateInterface& handle, double previous,
                                 const std::string& label) {
    if (!handle) {
      return previous;
    }
#if HARDWARE_INTERFACE_VERSION_MAJOR >= 4
    const auto value = handle->get().get_optional();
    if (!value.has_value() || !std::isfinite(value.value())) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                           "Invalid state interface %s; retaining previous value", label.c_str());
      return previous;
    }
    return value.value();
#else
    const double value = handle->get().get_value();
    if (!std::isfinite(value)) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                           "Invalid state interface %s; retaining previous value", label.c_str());
      return previous;
    }
    return value;
#endif
  };

  for (size_t index = 0; index < joints_.size(); ++index) {
    const auto& joint = joints_[index];
    auto& state = robot_state_.joints[index];
    state.position = read_value(joint->position_handle, state.position, joint->name + "/position");
    state.velocity = read_value(joint->velocity_handle, state.velocity, joint->name + "/velocity");
    state.effort = read_value(joint->effort_handle, state.effort, joint->name + "/effort");
    state.state_timestamp = time.seconds();
  }

  const std::array<double, 3> acceleration = imu_sensor_->get_linear_acceleration();
  const std::array<double, 3> angular_velocity = imu_sensor_->get_angular_velocity();
  const std::array<double, 4> orientation = imu_sensor_->get_orientation();
  const double quaternion_norm_sq =
      orientation[0] * orientation[0] + orientation[1] * orientation[1] +
      orientation[2] * orientation[2] + orientation[3] * orientation[3];
  if (!finiteArray(acceleration) || !finiteArray(angular_velocity) || !finiteArray(orientation) ||
      quaternion_norm_sq <= 1.0e-12) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "IMU state interfaces contain invalid data");
    return false;
  }
  robot_state_.imu.linear_acceleration = acceleration;
  robot_state_.imu.angular_velocity = angular_velocity;
  robot_state_.imu.orientation = {orientation[3], orientation[0], orientation[1], orientation[2]};

  robot_state_.timestamp = time.seconds();
  robot_state_.period = period.seconds();
  {
    std::lock_guard<std::mutex> lock(motion_cmd_mutex_);
    const std::uint64_t now_ns = steadyNowNanoseconds();
    const auto timeout_ns = static_cast<std::uint64_t>(motion_command_timeout_sec_ * 1.0e9);
    if (params_.use_dt7) {
      const double dt7_linear = read_value(dt7_cmd_vel_x_handle_, 0.0, "dt7/cmd_vel_x");
      const double dt7_yaw = read_value(dt7_cmd_omega_z_handle_, 0.0, "dt7/cmd_omega_z");
      const double dt7_height =
          read_value(dt7_cmd_height_handle_, params_.default_command_height, "dt7/cmd_height");
      geometry_msgs::msg::Twist dt7_motion;
      dt7_motion.linear.x = dt7_linear;
      dt7_motion.angular.z = dt7_yaw;
      const auto limited_motion = limitMotionCommand(dt7_motion, params_);
      latest_lin_vel_x_ = limited_motion.linear.x;
      latest_ang_vel_z_ = limited_motion.angular.z;
      latest_height_ = limitHeightCommand(dt7_height, params_);
      motion_cmd_received_ = true;
      height_cmd_received_ = true;
      last_motion_command_receive_ns_ = now_ns;
      last_height_command_receive_ns_ = now_ns;

      const int raw_state =
          static_cast<int>(std::llround(read_value(dt7_cmd_state_handle_, 0.0, "dt7/cmd_state")));
      if (raw_state != dt7_previous_state_) {
        ControllerState target = ControllerState::IDLE;
        if (resolveStateCommand(raw_state, target)) {
          state_machine_->setTargetState(target);
          RCLCPP_INFO(get_node()->get_logger(), "DT7 requested controller state %d", raw_state);
        } else {
          RCLCPP_WARN(get_node()->get_logger(),
                      "Ignored DT7 state %d; normal-only deployment accepts 0..3", raw_state);
        }
        dt7_previous_state_ = raw_state;
      }
    } else if (motion_cmd_received_ && (last_motion_command_receive_ns_ == 0 ||
                                        now_ns - last_motion_command_receive_ns_ > timeout_ns)) {
      latest_lin_vel_x_ = 0.0;
      latest_ang_vel_z_ = 0.0;
      motion_cmd_received_ = false;
      last_motion_command_receive_ns_ = 0;
      RCLCPP_WARN(get_node()->get_logger(),
                  "Motion command timed out after %.3f s; velocity command set to zero",
                  motion_command_timeout_sec_);
    }
    if (!params_.use_dt7 && height_cmd_received_ &&
        (last_height_command_receive_ns_ == 0 ||
         now_ns - last_height_command_receive_ns_ > timeout_ns)) {
      const ControllerState state = state_machine_->getCurrentState();
      if (state == ControllerState::INIT || state == ControllerState::IDLE) {
        latest_height_ = params_.default_command_height;
      }
      height_cmd_received_ = false;
      last_height_command_receive_ns_ = 0;
    }
    robot_state_.command.cmd_vel = {latest_lin_vel_x_, 0.0, latest_ang_vel_z_};
    robot_state_.command.cmd_height = latest_height_;
  }
  robot_state_.run();
  return true;
}

void TemplateRos2Controller::writeControllerCommands(ControllerState state) {
  const bool active = state == ControllerState::PREPARE || state == ControllerState::RL;
  const bool hardware_pd = params_.lowlevel_output_mode == "hardware_pd";
  const bool hardware_pd_velocity = params_.lowlevel_output_mode == "hardware_pd_vel";
  std::vector<double> final_torques(joints_.size(), 0.0);

  for (size_t index = 0; index < joints_.size(); ++index) {
    const auto& joint = joints_[index];
    auto& command = joint_command_snapshots_[index];
    command = JointCommandSnapshot{};
    command.position = robot_state_.joints[index].position;

    if (active) {
      const bool wheel = index == 4 || index == 5;
      if (hardware_pd_velocity && wheel) {
        command.position = 0.0;
        command.velocity =
            state == ControllerState::RL ? robot_state_.joints[index].output_velocity : 0.0;
      } else if (hardware_pd || hardware_pd_velocity) {
        command.position = robot_state_.joints[index].output_position;
      } else {
        command.velocity = robot_state_.joints[index].output_velocity;
        command.effort = robot_state_.joints[index].output_torque;
      }

      if (hardware_pd || hardware_pd_velocity) {
        const bool spring = index >= 6;
        if (spring || (wheel && hardware_pd)) {
          command.effort = robot_state_.joints[index].output_torque;
        }
      }
      if (hardware_pd || hardware_pd_velocity) {
        if (state == ControllerState::PREPARE && index < 4) {
          command.kp = index < params_.prepare_kp.size()
                           ? nonnegativeFinite(params_.prepare_kp[index])
                           : 0.0;
          command.kd = index < params_.prepare_kd.size()
                           ? nonnegativeFinite(params_.prepare_kd[index])
                           : 0.0;
        } else {
          command.kp = index < params_.joint_stiffness.size()
                           ? nonnegativeFinite(params_.joint_stiffness[index])
                           : 0.0;
          command.kd = index < params_.joint_damping.size()
                           ? nonnegativeFinite(params_.joint_damping[index])
                           : 0.0;
        }
      }
    }

    double raw_final_torque = command.effort;
    if (active && (hardware_pd || hardware_pd_velocity)) {
      raw_final_torque += command.kp * positionError(joint->name, command.position,
                                                     robot_state_.joints[index].position) +
                          command.kd * (command.velocity - robot_state_.joints[index].velocity);
    }
    raw_final_torque = finiteOrZero(raw_final_torque);
    if (index < params_.joint_output_min.size() && index < params_.joint_output_max.size() &&
        std::isfinite(params_.joint_output_min[index]) &&
        std::isfinite(params_.joint_output_max[index]) &&
        params_.joint_output_min[index] <= params_.joint_output_max[index]) {
      const double limited = std::clamp(raw_final_torque, params_.joint_output_min[index],
                                        params_.joint_output_max[index]);
      if (active && (hardware_pd || hardware_pd_velocity)) {
        command.effort += limited - raw_final_torque;
      } else {
        command.effort = limited;
      }
      final_torques[index] = limited;
    } else {
      final_torques[index] = raw_final_torque;
    }

    joint->position_command_handle->get().set_value(finiteOrZero(command.position));
    joint->velocity_command_handle->get().set_value(finiteOrZero(command.velocity));
    joint->effort_command_handle->get().set_value(finiteOrZero(command.effort));
    joint->kp_command_handle->get().set_value(nonnegativeFinite(command.kp));
    joint->kd_command_handle->get().set_value(nonnegativeFinite(command.kd));
  }

  if (joint_final_torque_publisher_) {
    std_msgs::msg::Float64MultiArray message;
    message.data = final_torques;
    joint_final_torque_publisher_->publish(message);
  }
  if (joint_command_publisher_) {
    sensor_msgs::msg::JointState message;
    message.header.stamp = last_update_time_;
    message.header.frame_id = "base_link";
    for (size_t index = 0; index < joints_.size(); ++index) {
      message.name.push_back(joints_[index]->name);
      message.position.push_back(joint_command_snapshots_[index].position);
      message.velocity.push_back(joint_command_snapshots_[index].velocity);
      message.effort.push_back(joint_command_snapshots_[index].effort);
    }
    joint_command_publisher_->publish(message);
  }
}

void TemplateRos2Controller::writeSafeJointCommands() {
  joint_command_snapshots_.resize(joints_.size());
  for (size_t index = 0; index < joints_.size(); ++index) {
    const auto& joint = joints_[index];
    const double position = index < robot_state_.joints.size()
                                ? finiteOrZero(robot_state_.joints[index].position)
                                : 0.0;
    if (joint->position_command_handle) {
      joint->position_command_handle->get().set_value(position);
    }
    if (joint->velocity_command_handle) {
      joint->velocity_command_handle->get().set_value(0.0);
    }
    if (joint->effort_command_handle) {
      joint->effort_command_handle->get().set_value(0.0);
    }
    if (joint->kp_command_handle) {
      joint->kp_command_handle->get().set_value(0.0);
    }
    if (joint->kd_command_handle) {
      joint->kd_command_handle->get().set_value(0.0);
    }
    joint_command_snapshots_[index] = JointCommandSnapshot{};
    joint_command_snapshots_[index].position = position;
    if (index < robot_state_.joints.size()) {
      robot_state_.joints[index].output_position = position;
      robot_state_.joints[index].output_velocity = 0.0;
      robot_state_.joints[index].output_torque = 0.0;
      robot_state_.joints[index].output_kp = 0.0;
      robot_state_.joints[index].output_kd = 0.0;
    }
  }
}

void TemplateRos2Controller::resetControllerRuntime(const rclcpp::Time& time) {
  if (state_machine_) {
    state_machine_->reset(robot_state_, time);
  }
  auto_enter_rl_pending_ = params_.auto_enter_rl;
  dt7_previous_state_ = -1;
  {
    std::lock_guard<std::mutex> lock(motion_cmd_mutex_);
    latest_lin_vel_x_ = 0.0;
    latest_ang_vel_z_ = 0.0;
    latest_height_ = params_.default_command_height;
    motion_cmd_received_ = false;
    height_cmd_received_ = false;
    last_motion_command_receive_ns_ = 0;
    last_height_command_receive_ns_ = 0;
    robot_state_.command.cmd_vel = {0.0, 0.0, 0.0};
    robot_state_.command.cmd_height = latest_height_;
  }
  writeSafeJointCommands();
}

void TemplateRos2Controller::motionCommandCallback(
    const geometry_msgs::msg::Twist::SharedPtr message) {
  if (!message || !std::isfinite(message->linear.x) || !std::isfinite(message->angular.z)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "Rejected non-finite motion command");
    return;
  }
  const auto limited = limitMotionCommand(*message, params_);
  if (limited.linear.x != message->linear.x || limited.angular.z != message->angular.z) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "Clamped motion command to linear.x [%.3f, %.3f], angular.z [%.3f, %.3f]",
                         params_.motion_linear_x_min, params_.motion_linear_x_max,
                         params_.motion_angular_z_min, params_.motion_angular_z_max);
  }
  std::lock_guard<std::mutex> lock(motion_cmd_mutex_);
  latest_lin_vel_x_ = limited.linear.x;
  latest_ang_vel_z_ = limited.angular.z;
  motion_cmd_received_ = true;
  last_motion_command_receive_ns_ = steadyNowNanoseconds();
}

void TemplateRos2Controller::heightCommandCallback(
    const std_msgs::msg::Float64::SharedPtr message) {
  if (!message || !std::isfinite(message->data)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "Rejected non-finite height command");
    return;
  }
  const double limited_height = limitHeightCommand(message->data, params_);
  if (limited_height != message->data) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "Clamped height command to [%.3f, %.3f] m", params_.command_height_min,
                         params_.command_height_max);
  }
  std::lock_guard<std::mutex> lock(motion_cmd_mutex_);
  latest_height_ = limited_height;
  height_cmd_received_ = true;
  last_height_command_receive_ns_ = steadyNowNanoseconds();
}

bool TemplateRos2Controller::resolveStateCommand(int raw_state, ControllerState& target) const {
  switch (raw_state) {
    case 0:
      target = ControllerState::INIT;
      return true;
    case 1:
      target = ControllerState::IDLE;
      return true;
    case 2:
      target = ControllerState::PREPARE;
      return true;
    case 3:
      target = rl_inference_ready_ ? ControllerState::RL : ControllerState::IDLE;
      return rl_inference_ready_;
    default:
      return false;
  }
}

void TemplateRos2Controller::stateCommandCallback(const std_msgs::msg::Int32::SharedPtr message) {
  if (!message || !state_machine_) {
    return;
  }
  ControllerState target = ControllerState::IDLE;
  if (!resolveStateCommand(message->data, target)) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Invalid state command %d (use 0=INIT, 1=IDLE, 2=PREPARE, 3=RL)", message->data);
    return;
  }
  state_machine_->setTargetState(target);
}

controller_interface::CallbackReturn TemplateRos2Controller::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  resetControllerRuntime(get_node()->now());
  for (auto& joint : joints_) {
    joint->position_command_handle.reset();
    joint->velocity_command_handle.reset();
    joint->effort_command_handle.reset();
    joint->kp_command_handle.reset();
    joint->kd_command_handle.reset();
    joint->position_handle.reset();
    joint->velocity_handle.reset();
    joint->effort_handle.reset();
  }
  dt7_cmd_state_handle_.reset();
  dt7_cmd_vel_x_handle_.reset();
  dt7_cmd_omega_z_handle_.reset();
  dt7_cmd_height_handle_.reset();
  dt7_previous_state_ = -1;
  imu_sensor_->release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TemplateRos2Controller::on_cleanup(
    const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  motion_cmd_subscriber_.reset();
  height_cmd_subscriber_.reset();
  state_cmd_subscriber_.reset();
  current_state_publisher_.reset();
  joint_command_publisher_.reset();
  joint_final_torque_publisher_.reset();
  joints_.clear();
  robot_state_ = RobotState{};
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TemplateRos2Controller::on_error(
    const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  resetControllerRuntime(get_node()->now());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TemplateRos2Controller::on_shutdown(
    const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  resetControllerRuntime(get_node()->now());
  return controller_interface::CallbackReturn::SUCCESS;
}

TemplateRos2Controller::~TemplateRos2Controller() { state_machine_.reset(); }

}  // namespace robot_locomotion

PLUGINLIB_EXPORT_CLASS(robot_locomotion::TemplateRos2Controller,
                       controller_interface::ControllerInterface)
