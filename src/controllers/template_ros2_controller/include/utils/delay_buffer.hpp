// Copyright (c) 2025 SCUTRobotLab
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef UTILS__DELAY_BUFFER_HPP_
#define UTILS__DELAY_BUFFER_HPP_

#include <deque>
#include <stdexcept>
#include <vector>

namespace robot_locomotion {

/**
 * @brief 观测数据延迟缓冲区模板类
 *
 * 该类实现了一个环形缓冲区，用于模拟传感器数据的延迟效果。
 * 通过在仿真中添加延迟来模拟硬件通信与执行链路的时延。
 *
 * @tparam T 数据类型（如 double, float, std::vector<double> 等）
 */
template <typename T>
class DelayBuffer {
 public:
  /**
   * @brief 构造函数
   * @param delay_steps 延迟步数（0 表示无延迟）
   */
  explicit DelayBuffer(size_t delay_steps = 0) : delay_steps_(delay_steps) {}

  /**
   * @brief 设置延迟步数
   * @param delay_steps 新的延迟步数
   */
  void setDelaySteps(size_t delay_steps) {
    delay_steps_ = delay_steps;
    // 清空缓冲区以避免旧数据干扰
    buffer_.clear();
  }

  /**
   * @brief 获取当前延迟步数
   * @return 延迟步数
   */
  size_t getDelaySteps() const { return delay_steps_; }

  /**
   * @brief 推入新数据并返回延迟后的数据
   * @param data 新的观测数据
   * @return 延迟后的数据（如果缓冲区未满，返回最早的数据）
   */
  T pushAndGet(const T& data) {
    // 如果延迟步数为 0，直接返回输入数据
    if (delay_steps_ == 0) {
      return data;
    }

    // 将新数据推入缓冲区
    buffer_.push_back(data);

    // 如果缓冲区未满，返回最早的数据（可能是首次输入或之前的数据）
    if (buffer_.size() <= delay_steps_) {
      return buffer_.front();
    }

    // 缓冲区已满，弹出最旧的数据并返回
    T delayed_data = buffer_.front();
    buffer_.pop_front();
    return delayed_data;
  }

  /**
   * @brief 清空缓冲区
   */
  void clear() { buffer_.clear(); }

  /**
   * @brief 检查缓冲区是否为空
   * @return 如果为空返回 true
   */
  bool isEmpty() const { return buffer_.empty(); }

  /**
   * @brief 获取当前缓冲区大小
   * @return 缓冲区中的数据数量
   */
  size_t size() const { return buffer_.size(); }

 private:
  size_t delay_steps_;    ///< 延迟步数
  std::deque<T> buffer_;  ///< 环形缓冲区（使用 deque 实现）
};

/**
 * @brief 观测向量延迟缓冲区（特化版本）
 *
 * 专用于 std::vector<float> 类型的延迟缓冲区，常用于 RL 网络输入观测。
 */
using ObservationDelayBuffer = DelayBuffer<std::vector<float>>;

}  // namespace robot_locomotion

#endif  // UTILS__DELAY_BUFFER_HPP_
