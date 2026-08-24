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

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "mujoco/mujoco.h"
#include "mujoco_ros2_control/mujoco_rendering.hpp"
#include "mujoco_ros2_control/mujoco_ros2_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

// MuJoCo data structures
mjModel* mujoco_model = nullptr;
mjData* mujoco_data = nullptr;

// main function
int main(int argc, const char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(
      "mujoco_ros2_control_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  RCLCPP_INFO_STREAM(node->get_logger(), "Initializing mujoco_ros2_control node...");
  auto model_path = node->get_parameter("mujoco_model_path").as_string();
  RCLCPP_INFO_STREAM(node->get_logger(), "Loading MuJoCo model from: " << model_path);
  bool render = true;
  bool headless_real_time = true;
  double run_duration = 0.0;
  if (!node->has_parameter("render")) {
    node->declare_parameter<bool>("render", true);
  }
  if (!node->has_parameter("run_duration")) {
    node->declare_parameter<double>("run_duration", 0.0);
  }
  if (!node->has_parameter("headless_real_time")) {
    node->declare_parameter<bool>("headless_real_time", true);
  }
  render = node->get_parameter("render").as_bool();
  headless_real_time = node->get_parameter("headless_real_time").as_bool();
  run_duration = node->get_parameter("run_duration").as_double();

  // load and compile model
  char error[1000] = "Could not load binary model";
  if (std::strlen(model_path.c_str()) > 4 &&
      !std::strcmp(model_path.c_str() + std::strlen(model_path.c_str()) - 4, ".mjb")) {
    mujoco_model = mj_loadModel(model_path.c_str(), 0);
  } else {
    mujoco_model = mj_loadXML(model_path.c_str(), 0, error, 1000);
  }
  if (!mujoco_model) {
    RCLCPP_FATAL(node->get_logger(), "Failed to load MuJoCo model: %s", error);
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco model has been successfully loaded !");
  // make data
  mujoco_data = mj_makeData(mujoco_model);
  if (!mujoco_data) {
    RCLCPP_FATAL(node->get_logger(), "Failed to allocate MuJoCo simulation data");
    mj_deleteModel(mujoco_model);
    mujoco_model = nullptr;
    rclcpp::shutdown();
    return 1;
  }

  // initialize mujoco control
  auto mujoco_control =
      std::make_unique<mujoco_ros2_control::MujocoRos2Control>(node, mujoco_model, mujoco_data);

  bool control_initialized = false;
  try {
    control_initialized = mujoco_control->init();
  } catch (const std::exception& exception) {
    RCLCPP_FATAL(node->get_logger(), "MuJoCo ros2_control initialization threw an exception: %s",
                 exception.what());
  }
  if (!control_initialized) {
    RCLCPP_FATAL(node->get_logger(), "MuJoCo ros2_control initialization failed");
    mujoco_control.reset();
    mj_deleteData(mujoco_data);
    mj_deleteModel(mujoco_model);
    mujoco_data = nullptr;
    mujoco_model = nullptr;
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO_STREAM(node->get_logger(),
                     "Mujoco ros2 controller has been successfully initialized !");

  auto rendering = mujoco_ros2_control::MujocoRendering::get_instance();
  if (render) {
    // Initialize the interactive MuJoCo viewer.
    if (!glfwInit()) {
      mju_error("Could not initialize GLFW");
    }
    rendering->init(mujoco_model, mujoco_data);
    RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco rendering has been successfully initialized !");
  } else {
    RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco rendering disabled (headless mode)");
  }

  // Run the main loop, targeting real-time simulation and 60 fps rendering.
  auto next_headless_step = std::chrono::steady_clock::now();
  auto reset_simulation = [&]() {
    mj_resetData(mujoco_model, mujoco_data);
    mj_forward(mujoco_model, mujoco_data);
    mujoco_control->reset_sim_time();
    next_headless_step = std::chrono::steady_clock::now();
    RCLCPP_INFO_STREAM(node->get_logger(), "MuJoCo simulation reset");
  };
  std::atomic_bool reset_requested{false};
  auto reset_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  reset_qos.best_effort();
  reset_qos.durability_volatile();
  auto reset_subscriber = node->create_subscription<std_msgs::msg::Empty>(
      "reload_robot", reset_qos, [&](const std_msgs::msg::Empty::SharedPtr) {
        reset_requested.store(true);
        RCLCPP_INFO_STREAM(node->get_logger(), "MuJoCo reset requested from ROS topic");
      });
  (void)reset_subscriber;

  while (rclcpp::ok() && (!render || !rendering->is_close_flag_raised()) &&
         (run_duration <= 0.0 || mujoco_data->time < run_duration)) {
    rclcpp::spin_some(node);

    if (reset_requested.exchange(false) || (render && rendering->consume_reset_request())) {
      reset_simulation();
    }

    if (!render) {
      mujoco_control->update();
      if (headless_real_time) {
        next_headless_step += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(mujoco_model->opt.timestep));
        std::this_thread::sleep_until(next_headless_step);
      }
      continue;
    }

    if (rendering->is_paused()) {
      rclcpp::spin_some(node);
      rendering->update();
      if (headless_real_time) {
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
      }
      continue;
    }

    // advance interactive simulation for 1/60 sec
    mjtNum simstart = mujoco_data->time;
    auto frame_wall_start = std::chrono::steady_clock::now();
    while (mujoco_data->time - simstart < 1.0 / 60.0) {
      mujoco_control->update();
    }
    rclcpp::spin_some(node);
    rendering->update();

    if (headless_real_time) {
      auto frame_sim_duration = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(mujoco_data->time - simstart));
      std::this_thread::sleep_until(frame_wall_start + frame_sim_duration);
    }
  }

  if (render) {
    rendering->close();
  }

  // free MuJoCo model and data
  mujoco_control.reset();
  mj_deleteData(mujoco_data);
  mj_deleteModel(mujoco_model);

  return 0;
}
