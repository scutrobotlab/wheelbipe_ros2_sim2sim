// Copyright (c) 2025 SCUTRobotLab
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef UTILS__NOISE_GENERATOR_HPP_
#define UTILS__NOISE_GENERATOR_HPP_

#include <array>
#include <random>
#include <vector>

namespace robot_locomotion {

/**
 * @brief 高斯噪声生成器，用于模拟传感器噪声
 *
 * 该类持有独立随机数生成器，可为 IMU、关节状态等传感器数据添加噪声。
 * 实例应由单一控制线程使用；固定 seed 可用于复现实验输入。
 */
class NoiseGenerator {
 public:
  /**
   * @brief 构造函数，使用随机种子初始化
   */
  NoiseGenerator();

  /**
   * @brief 构造函数，使用指定种子初始化
   * @param seed 随机数生成器种子
   */
  explicit NoiseGenerator(unsigned int seed);

  /**
   * @brief 生成标量高斯噪声
   * @param mean 均值（默认为 0）
   * @param stddev 标准差
   * @return 生成的噪声值
   */
  double generateGaussianNoise(double mean, double stddev);

  /**
   * @brief 为 std::array 添加高斯噪声
   * @tparam N 数组大小
   * @param data 输入数据（会被修改）
   * @param stddev 每个元素的标准差数组
   */
  template <size_t N>
  void addGaussianNoise(std::array<double, N>& data, const std::array<double, N>& stddev) {
    for (size_t i = 0; i < N; ++i) {
      data[i] += generateGaussianNoise(0.0, stddev[i]);
    }
  }

  /**
   * @brief 为 std::vector 添加高斯噪声
   * @param data 输入数据（会被修改）
   * @param stddev 标准差（所有元素使用相同标准差）
   */
  void addGaussianNoise(std::vector<double>& data, double stddev);

  /**
   * @brief 为单个 double 值添加高斯噪声
   * @param value 输入值（会被修改）
   * @param stddev 标准差
   */
  void addGaussianNoise(double& value, double stddev);

  /**
   * @brief 重新设置随机数生成器种子
   * @param seed 新的种子值
   */
  void setSeed(unsigned int seed);

 private:
  std::mt19937 generator_;  ///< 梅森旋转算法随机数生成器
};

}  // namespace robot_locomotion

#endif  // UTILS__NOISE_GENERATOR_HPP_
