// Copyright (c) 2025 SCUTRobotLab
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "onnxruntime/onnxruntime_inference.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <numeric>

namespace robot_locomotion {
namespace {

size_t fixedElementCount(const std::vector<int64_t>& shape) {
  if (shape.empty() ||
      std::any_of(shape.begin(), shape.end(), [](int64_t value) { return value <= 0; })) {
    return 0;
  }
  return std::accumulate(shape.begin(), shape.end(), size_t{1},
                         [](size_t lhs, int64_t rhs) { return lhs * static_cast<size_t>(rhs); });
}

}  // namespace

ONNXRuntimeInference::ONNXRuntimeInference(rclcpp::Logger logger)
    : logger_(logger), memory_info_(nullptr) {}

ONNXRuntimeInference::~ONNXRuntimeInference() {
  stop();
  session_.reset();
  env_.reset();
}

bool ONNXRuntimeInference::initialize(const std::string& model_path, int inference_frequency_hz) {
  if (initialized_.load()) {
    RCLCPP_WARN(logger_, "ONNX Runtime inference is already initialized");
    return true;
  }
  if (!std::filesystem::is_regular_file(model_path)) {
    RCLCPP_ERROR(logger_, "ONNX policy does not exist: %s", model_path.c_str());
    return false;
  }

  model_path_ = model_path;
  inference_frequency_hz_ = std::max(1, inference_frequency_hz);
  if (!createSession()) {
    return false;
  }

  input_data_.assign(input_size_, 0.0F);
  output_data_.assign(output_size_, 0.0F);
  output_ready_ = false;
  initialized_.store(true);
  RCLCPP_INFO(logger_, "ONNX Runtime CPU policy ready: %s ([1, %zu] -> [1, %zu], %d Hz)",
              model_path_.c_str(), input_size_, output_size_, inference_frequency_hz_);
  return true;
}

bool ONNXRuntimeInference::createSession() {
  try {
    env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "wheelbipe_policy");
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetInterOpNumThreads(1);
    session_ = std::make_unique<Ort::Session>(*env_, model_path_.c_str(), session_options_);

    if (session_->GetInputCount() != 1 || session_->GetOutputCount() != 1) {
      RCLCPP_ERROR(
          logger_,
          "The baseline policy must expose exactly one input and one output; got %zu and %zu",
          session_->GetInputCount(), session_->GetOutputCount());
      return false;
    }

    Ort::AllocatorWithDefaultOptions allocator;
    const auto actual_input = session_->GetInputNameAllocated(0, allocator);
    const auto actual_output = session_->GetOutputNameAllocated(0, allocator);
    const std::string actual_input_name = actual_input.get();
    const std::string actual_output_name = actual_output.get();
    if (input_tensor_name_ != actual_input_name) {
      RCLCPP_ERROR(logger_, "Policy input tensor must be '%s'; model exposes '%s'",
                   input_tensor_name_.c_str(), actual_input_name.c_str());
      return false;
    }
    if (output_tensor_name_ != actual_output_name) {
      RCLCPP_ERROR(logger_, "Policy output tensor must be '%s'; model exposes '%s'",
                   output_tensor_name_.c_str(), actual_output_name.c_str());
      return false;
    }

    // TensorTypeAndShapeInfo is an unowned view of its TypeInfo. Keep both
    // owning TypeInfo objects alive until all type and shape queries finish.
    const auto input_type_info = session_->GetInputTypeInfo(0);
    const auto output_type_info = session_->GetOutputTypeInfo(0);
    const auto input_info = input_type_info.GetTensorTypeAndShapeInfo();
    const auto output_info = output_type_info.GetTensorTypeAndShapeInfo();
    if (input_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        output_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
      RCLCPP_ERROR(logger_, "The policy input and output must use float32 tensors");
      return false;
    }

    input_shape_ = input_info.GetShape();
    const auto output_shape = output_info.GetShape();
    input_size_ = fixedElementCount(input_shape_);
    output_size_ = fixedElementCount(output_shape);
    if (input_shape_ != std::vector<int64_t>({1, 35}) ||
        output_shape != std::vector<int64_t>({1, 6})) {
      RCLCPP_ERROR(
          logger_,
          "Policy tensor contract mismatch: expected [1,35] -> [1,6], got %zu -> %zu elements",
          input_size_, output_size_);
      return false;
    }

    memory_info_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    return true;
  } catch (const Ort::Exception& error) {
    RCLCPP_ERROR(logger_, "Failed to create ONNX Runtime session: %s", error.what());
    return false;
  }
}

void ONNXRuntimeInference::start() {
  if (!initialized_.load()) {
    RCLCPP_ERROR(logger_, "Cannot start an uninitialized ONNX Runtime policy");
    return;
  }
  running_.store(true);
}

void ONNXRuntimeInference::stop() {
  running_.store(false);
  std::lock_guard<std::mutex> lock(data_mutex_);
  output_ready_ = false;
}

bool ONNXRuntimeInference::setInput(const std::vector<float>& input_data) {
  if (!initialized_.load() || !running_.load() || input_data.size() != input_size_ ||
      !std::all_of(input_data.begin(), input_data.end(),
                   [](float value) { return std::isfinite(value); })) {
    return false;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  input_data_ = input_data;
  return runLocked();
}

bool ONNXRuntimeInference::runLocked() {
  try {
    const auto start_time = std::chrono::steady_clock::now();
    auto input_tensor =
        Ort::Value::CreateTensor<float>(memory_info_, input_data_.data(), input_data_.size(),
                                        input_shape_.data(), input_shape_.size());
    const char* input_names[] = {input_tensor_name_.c_str()};
    const char* output_names[] = {output_tensor_name_.c_str()};
    auto outputs =
        session_->Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);
    if (outputs.size() != 1 || !outputs[0].IsTensor()) {
      RCLCPP_ERROR(logger_, "ONNX Runtime returned an invalid policy output");
      return false;
    }

    const auto output_info = outputs[0].GetTensorTypeAndShapeInfo();
    if (output_info.GetElementCount() != output_size_) {
      RCLCPP_ERROR(logger_, "ONNX Runtime output size changed: expected %zu, got %zu", output_size_,
                   output_info.GetElementCount());
      return false;
    }
    const float* output = outputs[0].GetTensorData<float>();
    output_data_.assign(output, output + output_size_);
    output_ready_ = true;

    if (print_inference_time_.load()) {
      const auto elapsed =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time)
              .count();
      RCLCPP_INFO(logger_, "ONNX Runtime CPU inference: %.3f ms", elapsed);
    }
    return true;
  } catch (const Ort::Exception& error) {
    output_ready_ = false;
    RCLCPP_ERROR(logger_, "ONNX Runtime inference failed: %s", error.what());
    return false;
  }
}

bool ONNXRuntimeInference::getOutput(std::vector<float>& output_data) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!output_ready_) {
    return false;
  }
  output_data = output_data_;
  output_ready_ = false;
  return true;
}

}  // namespace robot_locomotion
