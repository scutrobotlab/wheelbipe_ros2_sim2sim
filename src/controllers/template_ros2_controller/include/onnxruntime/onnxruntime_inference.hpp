// Copyright (c) 2025 SCUTRobotLab
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ONNXRUNTIME__ONNXRUNTIME_INFERENCE_HPP_
#define ONNXRUNTIME__ONNXRUNTIME_INFERENCE_HPP_

#include <onnxruntime_cxx_api.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace robot_locomotion {

/// Synchronous CPU inference for the bundled [1, 35] -> [1, 6] policy.
class ONNXRuntimeInference {
 public:
  explicit ONNXRuntimeInference(rclcpp::Logger logger);
  ~ONNXRuntimeInference();

  bool initialize(const std::string& model_path, int inference_frequency_hz);

  void start();
  void stop();

  bool setInput(const std::vector<float>& input_data);
  bool getOutput(std::vector<float>& output_data);

  bool isInitialized() const { return initialized_.load(); }
  bool isRunning() const { return running_.load(); }
  size_t getInputElementCount() const { return input_size_; }
  size_t getOutputElementCount() const { return output_size_; }

  void setPrintInferenceTime(bool print) { print_inference_time_.store(print); }

 private:
  bool createSession();
  bool runLocked();

  rclcpp::Logger logger_;
  std::string model_path_;
  std::string input_tensor_name_{"obs"};
  std::string output_tensor_name_{"actions"};
  int inference_frequency_hz_ = 50;

  std::unique_ptr<Ort::Env> env_;
  std::unique_ptr<Ort::Session> session_;
  Ort::SessionOptions session_options_;
  Ort::MemoryInfo memory_info_;
  std::vector<int64_t> input_shape_;
  size_t input_size_ = 0;
  size_t output_size_ = 0;

  mutable std::mutex data_mutex_;
  std::vector<float> input_data_;
  std::vector<float> output_data_;
  bool output_ready_ = false;

  std::atomic<bool> initialized_{false};
  std::atomic<bool> running_{false};
  std::atomic<bool> print_inference_time_{false};
};

}  // namespace robot_locomotion

#endif  // ONNXRUNTIME__ONNXRUNTIME_INFERENCE_HPP_
