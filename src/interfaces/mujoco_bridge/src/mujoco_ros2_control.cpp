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

#include "mujoco_ros2_control/mujoco_ros2_control.hpp"

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/version.h"

namespace mujoco_ros2_control {
namespace {
constexpr const char* kBootstrapRobotDescription =
    "<robot name=\"mujoco_ros2_control\">"
    "<link name=\"mujoco_control_bootstrap_link\"/>"
    "<ros2_control name=\"MujocoControlBootstrap\" type=\"system\">"
    "<hardware><plugin>mock_components/GenericSystem</plugin></hardware>"
    "</ros2_control>"
    "</robot>";
}  // namespace

MujocoRos2Control::MujocoRos2Control(rclcpp::Node::SharedPtr& node, mjModel* mujoco_model,
                                     mjData* mujoco_data)
    : node_(node),
      mj_model_(mujoco_model),
      mj_data_(mujoco_data),
      logger_(rclcpp::get_logger(node_->get_name() + std::string(".mujoco_ros2_control"))),
      control_period_(rclcpp::Duration(1, 0)),
      last_update_sim_time_ros_(0, 0, RCL_ROS_TIME) {}

MujocoRos2Control::~MujocoRos2Control() {
  stop_cm_thread_.store(true);
  if (cm_executor_) {
    cm_executor_->cancel();
  }

  if (cm_thread_.joinable()) {
    cm_thread_.join();
  }

  if (cm_executor_ && controller_manager_) {
    try {
      cm_executor_->remove_node(controller_manager_);
    } catch (const std::exception&) {
      // The controller manager can already be detached during ROS shutdown.
    }
  }
}

std::string MujocoRos2Control::get_robot_description() {
  // Getting robot description from parameter first. If not set trying from topic
  std::string robot_description;

  auto node = std::make_shared<rclcpp::Node>(
      "robot_description_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  if (node->has_parameter("robot_description")) {
    robot_description = node->get_parameter("robot_description").as_string();
    return robot_description;
  }

  RCLCPP_WARN(
      logger_,
      "Failed to get robot_description from parameter. Will listen on the ~/robot_description "
      "topic...");

  auto robot_description_sub = node->create_subscription<std_msgs::msg::String>(
      "robot_description", rclcpp::QoS(1).transient_local(),
      [&](const std_msgs::msg::String::SharedPtr msg) {
        if (!msg->data.empty() && robot_description.empty()) robot_description = msg->data;
      });

  while (robot_description.empty() && rclcpp::ok()) {
    rclcpp::spin_some(node);
    RCLCPP_INFO(node->get_logger(), "Waiting for robot description message");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  return robot_description;
}

bool MujocoRos2Control::init() {
  initialized_ = false;
  clock_publisher_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

  std::string urdf_string = this->get_robot_description();
  if (urdf_string.empty()) {
    RCLCPP_ERROR(logger_, "robot_description is empty; cannot initialize ros2_control");
    return false;
  }

  // setup actuators and mechanism control node.
  std::vector<hardware_interface::HardwareInfo> control_hardware_info;
  try {
    control_hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_string);
  } catch (const std::runtime_error& ex) {
    RCLCPP_ERROR_STREAM(logger_, "Error parsing URDF : " << ex.what());
    return false;
  }
  if (control_hardware_info.empty()) {
    RCLCPP_ERROR(logger_, "robot_description contains no ros2_control hardware components");
    return false;
  }

  try {
    robot_hw_sim_loader_.reset(new pluginlib::ClassLoader<MujocoSystemInterface>(
        "mujoco_ros2_control", "mujoco_ros2_control::MujocoSystemInterface"));
  } catch (pluginlib::LibraryLoadException& ex) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to create hardware interface loader:  " << ex.what());
    return false;
  }

  cm_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
#if HARDWARE_INTERFACE_VERSION_MAJOR >= 4
  hardware_interface::ResourceManagerParams rm_params;
  rm_params.robot_description = kBootstrapRobotDescription;
  rm_params.clock = node_->get_clock();
  rm_params.logger = logger_;
  rm_params.node_namespace = node_->get_namespace();
  rm_params.executor = cm_executor_;
  auto resource_manager = std::make_unique<hardware_interface::ResourceManager>(rm_params, true);
#else
  auto resource_manager =
      std::make_unique<hardware_interface::ResourceManager>(kBootstrapRobotDescription, true);
#endif

  for (const auto& hardware : control_hardware_info) {
#if HARDWARE_INTERFACE_VERSION_MAJOR >= 4
    std::string robot_hw_sim_type_str_ = hardware.hardware_plugin_name;
#else
    std::string robot_hw_sim_type_str_ = hardware.hardware_class_type;
#endif
    std::unique_ptr<MujocoSystemInterface> mujoco_system;
    try {
      mujoco_system = std::unique_ptr<MujocoSystemInterface>(
          robot_hw_sim_loader_->createUnmanagedInstance(robot_hw_sim_type_str_));
    } catch (pluginlib::PluginlibException& ex) {
      RCLCPP_ERROR_STREAM(logger_, "The plugin failed to load. Error: " << ex.what());
      return false;
    }

    urdf::Model urdf_model;
    if (!urdf_model.initString(urdf_string)) {
      RCLCPP_ERROR(logger_, "Failed to parse robot_description as a URDF model");
      return false;
    }
    if (!mujoco_system->init_sim(mj_model_, mj_data_, urdf_model, hardware)) {
      RCLCPP_FATAL(logger_, "Could not initialize robot simulation interface");
      return false;
    }

    resource_manager->import_component(std::move(mujoco_system), hardware);

    rclcpp_lifecycle::State state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                  hardware_interface::lifecycle_state_names::ACTIVE);
    resource_manager->set_component_state(hardware.name, state);
  }

  // Create the controller manager
  RCLCPP_INFO(logger_, "Loading controller_manager");
  auto cm_options = controller_manager::get_cm_node_options();
  // ResourceManager already owns the parsed description, so isolate the
  // controller manager's compatibility subscription from the public topic.
  cm_options.arguments(
      {"--ros-args", "-r", "robot_description:=mujoco_control_robot_description_internal"});
  controller_manager_ = std::make_shared<controller_manager::ControllerManager>(
      std::move(resource_manager), cm_executor_, "controller_manager", node_->get_namespace(),
      cm_options);
  cm_executor_->add_node(controller_manager_);

  if (!controller_manager_->has_parameter("update_rate")) {
    RCLCPP_ERROR_STREAM(logger_, "controller manager doesn't have an update_rate parameter");
    return false;
  }

  auto update_rate = controller_manager_->get_parameter("update_rate").as_int();
  if (update_rate <= 0) {
    RCLCPP_ERROR_STREAM(logger_,
                        "controller manager update_rate must be positive, got " << update_rate);
    return false;
  }
  control_period_ = rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / static_cast<double>(update_rate))));

  // Force setting of use_sime_time parameter
  controller_manager_->set_parameter(
      rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));

  stop_cm_thread_.store(false);
  auto spin = [this]() {
    while (rclcpp::ok() && !stop_cm_thread_.load()) {
      cm_executor_->spin_once();
    }
  };
  cm_thread_ = std::thread(spin);

  // 与 launch / controllers yaml 叠加以免 ParameterAlreadyDeclaredException
  if (!node_->has_parameter("publish_imu")) {
    node_->declare_parameter<bool>("publish_imu", true);
  }
  if (!node_->has_parameter("imu_topic")) {
    node_->declare_parameter<std::string>("imu_topic", "imu");
  }
  if (!node_->has_parameter("imu_frame_id")) {
    node_->declare_parameter<std::string>("imu_frame_id", "imu");
  }
  publish_imu_ = node_->get_parameter("publish_imu").as_bool();
  imu_frame_id_ = node_->get_parameter("imu_frame_id").as_string();
  const std::string imu_topic = node_->get_parameter("imu_topic").as_string();
  // 与 MJCF 中 <framequat name="imu"/>、<gyro name="gyro"/>、<accelerometer name="accelerometer"/>
  // 一致
  const int iq = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu");
  const int ig = mj_name2id(mj_model_, mjOBJ_SENSOR, "gyro");
  const int ia = mj_name2id(mj_model_, mjOBJ_SENSOR, "accelerometer");
  if (publish_imu_) {
    if (iq < 0 || ig < 0 || ia < 0) {
      RCLCPP_WARN(logger_,
                  "publish_imu enabled but MJCF sensors imu/gyro/accelerometer not all found "
                  "(quat=%d gyro=%d accel=%d). IMU topic will not be published.",
                  iq, ig, ia);
      publish_imu_ = false;
    } else {
      imu_quat_sensor_adr_ = mj_model_->sensor_adr[iq];
      gyro_sensor_adr_ = mj_model_->sensor_adr[ig];
      accel_sensor_adr_ = mj_model_->sensor_adr[ia];
      imu_publisher_ =
          node_->create_publisher<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS());
      RCLCPP_INFO(logger_, "Publishing sensor_msgs/Imu on '%s' (frame_id=%s)", imu_topic.c_str(),
                  imu_frame_id_.c_str());
    }
  }

  if (!node_->has_parameter("publish_base_state")) {
    node_->declare_parameter<bool>("publish_base_state", true);
  }
  if (!node_->has_parameter("base_state_topic")) {
    node_->declare_parameter<std::string>("base_state_topic", "mujoco_base_state");
  }
  publish_base_state_ = node_->get_parameter("publish_base_state").as_bool();
  if (publish_base_state_) {
    base_body_id_ = mj_name2id(mj_model_, mjOBJ_BODY, "base_link");
    if (base_body_id_ < 0) {
      RCLCPP_WARN(logger_, "publish_base_state enabled but MuJoCo body 'base_link' was not found.");
      publish_base_state_ = false;
    } else {
      const std::string base_state_topic = node_->get_parameter("base_state_topic").as_string();
      base_state_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
          base_state_topic, rclcpp::SensorDataQoS());
      RCLCPP_INFO(logger_, "Publishing [base_z, base_vz] on '%s'", base_state_topic.c_str());
    }
  }

  if (!node_->has_parameter("publish_wheel_contact_force")) {
    node_->declare_parameter<bool>("publish_wheel_contact_force", true);
  }
  if (!node_->has_parameter("wheel_contact_force_topic")) {
    node_->declare_parameter<std::string>("wheel_contact_force_topic",
                                          "wheel_ground_contact_force");
  }
  if (!node_->has_parameter("left_wheel_contact_geom")) {
    node_->declare_parameter<std::string>("left_wheel_contact_geom", "left_wheel_link_collision");
  }
  if (!node_->has_parameter("right_wheel_contact_geom")) {
    node_->declare_parameter<std::string>("right_wheel_contact_geom", "right_wheel_link_collision");
  }
  publish_wheel_contact_force_ = node_->get_parameter("publish_wheel_contact_force").as_bool();
  if (publish_wheel_contact_force_) {
    const std::array<std::string, 2> wheel_geom_names{
        {node_->get_parameter("left_wheel_contact_geom").as_string(),
         node_->get_parameter("right_wheel_contact_geom").as_string()}};
    for (size_t i = 0; i < wheel_geom_names.size(); ++i) {
      wheel_geom_ids_[i] = mj_name2id(mj_model_, mjOBJ_GEOM, wheel_geom_names[i].c_str());
    }
    if (wheel_geom_ids_[0] < 0 || wheel_geom_ids_[1] < 0) {
      RCLCPP_WARN(logger_,
                  "Wheel contact force publishing enabled but wheel collision geoms were not found "
                  "(left='%s' id=%d, right='%s' id=%d). Topic will not be published.",
                  wheel_geom_names[0].c_str(), wheel_geom_ids_[0], wheel_geom_names[1].c_str(),
                  wheel_geom_ids_[1]);
      publish_wheel_contact_force_ = false;
    } else {
      const std::string topic = node_->get_parameter("wheel_contact_force_topic").as_string();
      wheel_contact_force_publisher_ =
          node_->create_publisher<std_msgs::msg::Float64MultiArray>(topic, rclcpp::SensorDataQoS());
      RCLCPP_INFO(logger_,
                  "Publishing wheel-ground contact force magnitudes [left, right] in N on '%s'",
                  topic.c_str());
    }
  }

  initialized_ = true;
  return true;
}

void MujocoRos2Control::update() {
  if (!initialized_ || !controller_manager_ || !mj_model_ || !mj_data_) {
    RCLCPP_ERROR_THROTTLE(logger_, *node_->get_clock(), 1000,
                          "MuJoCo ros2_control update requested before successful initialization");
    return;
  }
  // Get the simulation time and period
  auto sim_time = mj_data_->time;
  int sim_time_sec = static_cast<int>(sim_time);
  int sim_time_nanosec = static_cast<int>((sim_time - sim_time_sec) * 1000000000);

  rclcpp::Time sim_time_ros(sim_time_sec, sim_time_nanosec, RCL_ROS_TIME);
  rclcpp::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  publish_sim_time(sim_time_ros);

  mj_step1(mj_model_, mj_data_);

  if (sim_period >= control_period_) {
    controller_manager_->read(sim_time_ros, sim_period);
    controller_manager_->update(sim_time_ros, sim_period);
    last_update_sim_time_ros_ = sim_time_ros;
    publish_imu_if_enabled(sim_time_ros);
    publish_base_state_if_enabled();
  }

  // use same time as for read and update call - this is how it is done in ros2_control_node
  controller_manager_->write(sim_time_ros, sim_period);

  mj_step2(mj_model_, mj_data_);

  if (sim_period >= control_period_) {
    publish_wheel_contact_force_if_enabled();
  }
}

void MujocoRos2Control::reset_sim_time() {
  if (!initialized_ || !mj_data_) {
    return;
  }
  const auto sim_time = mj_data_->time;
  const int sim_time_sec = static_cast<int>(sim_time);
  const int sim_time_nanosec = static_cast<int>((sim_time - sim_time_sec) * 1000000000);
  last_update_sim_time_ros_ = rclcpp::Time(sim_time_sec, sim_time_nanosec, RCL_ROS_TIME);
  publish_sim_time(last_update_sim_time_ros_);
}

void MujocoRos2Control::publish_sim_time(rclcpp::Time sim_time) {
  rosgraph_msgs::msg::Clock sim_time_msg;
  sim_time_msg.clock = sim_time;
  clock_publisher_->publish(sim_time_msg);
}

void MujocoRos2Control::publish_imu_if_enabled(const rclcpp::Time& sim_time) {
  if (!publish_imu_ || !imu_publisher_ || imu_quat_sensor_adr_ < 0) {
    return;
  }
  const mjtNum* s = mj_data_->sensordata;
  const int iq = imu_quat_sensor_adr_;
  const int ig = gyro_sensor_adr_;
  const int ia = accel_sensor_adr_;

  sensor_msgs::msg::Imu msg;
  msg.header.stamp = sim_time;
  msg.header.frame_id = imu_frame_id_;
  // framequat: [w,x,y,z]
  msg.orientation.w = s[iq];
  msg.orientation.x = s[iq + 1];
  msg.orientation.y = s[iq + 2];
  msg.orientation.z = s[iq + 3];
  msg.angular_velocity.x = s[ig];
  msg.angular_velocity.y = s[ig + 1];
  msg.angular_velocity.z = s[ig + 2];
  msg.linear_acceleration.x = s[ia];
  msg.linear_acceleration.y = s[ia + 1];
  msg.linear_acceleration.z = s[ia + 2];

  msg.orientation_covariance[0] = -1.0;
  msg.angular_velocity_covariance[0] = -1.0;
  msg.linear_acceleration_covariance[0] = -1.0;

  imu_publisher_->publish(msg);
}

void MujocoRos2Control::publish_base_state_if_enabled() {
  if (!publish_base_state_ || !base_state_publisher_ || base_body_id_ < 0) {
    return;
  }

  mjtNum velocity[6];
  mj_objectVelocity(mj_model_, mj_data_, mjOBJ_BODY, base_body_id_, velocity, 0);

  std_msgs::msg::Float64MultiArray msg;
  msg.data = {mj_data_->xpos[3 * base_body_id_ + 2],
              // mj_objectVelocity returns rotational components first, then linear components.
              velocity[5]};
  base_state_publisher_->publish(msg);
}

double MujocoRos2Control::wheel_ground_contact_force_magnitude(int wheel_geom_id) const {
  mjtNum total_force_world[3] = {0.0, 0.0, 0.0};
  for (int contact_index = 0; contact_index < mj_data_->ncon; ++contact_index) {
    const mjContact& contact = mj_data_->contact[contact_index];
    const bool wheel_is_geom1 = contact.geom1 == wheel_geom_id;
    const bool wheel_is_geom2 = contact.geom2 == wheel_geom_id;
    if (!wheel_is_geom1 && !wheel_is_geom2) {
      continue;
    }

    const int other_geom_id = wheel_is_geom1 ? contact.geom2 : contact.geom1;
    if (other_geom_id < 0 || mj_model_->geom_bodyid[other_geom_id] != 0) {
      // Static floor and terrain geoms belong to MuJoCo's world body (body 0).
      // Exclude contacts with other robot bodies.
      continue;
    }

    mjtNum contact_wrench[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    mj_contactForce(mj_model_, mj_data_, contact_index, contact_wrench);

    mjtNum force_world[3];
    // mj_contactForce returns force in the contact frame. contact.frame stores
    // the contact axes as rows, so its transpose maps contact-frame vectors to world.
    mju_mulMatTVec3(force_world, contact.frame, contact_wrench);
    const mjtNum sign = wheel_is_geom1 ? 1.0 : -1.0;
    for (int axis = 0; axis < 3; ++axis) {
      total_force_world[axis] += sign * force_world[axis];
    }
  }

  return mju_norm3(total_force_world);
}

void MujocoRos2Control::publish_wheel_contact_force_if_enabled() {
  if (!publish_wheel_contact_force_ || !wheel_contact_force_publisher_) {
    return;
  }

  std_msgs::msg::Float64MultiArray msg;
  msg.layout.dim.resize(1);
  msg.layout.dim[0].label = "left_wheel,right_wheel [N]";
  msg.layout.dim[0].size = 2;
  msg.layout.dim[0].stride = 2;
  msg.data = {wheel_ground_contact_force_magnitude(wheel_geom_ids_[0]),
              wheel_ground_contact_force_magnitude(wheel_geom_ids_[1])};
  wheel_contact_force_publisher_->publish(msg);
}

}  // namespace mujoco_ros2_control
