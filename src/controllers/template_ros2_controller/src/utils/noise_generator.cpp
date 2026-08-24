// Copyright (c) 2025 SCUTRobotLab
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "utils/noise_generator.hpp"

namespace robot_locomotion {

NoiseGenerator::NoiseGenerator() {
  // 使用随机设备生成种子
  std::random_device rd;
  generator_.seed(rd());
}

NoiseGenerator::NoiseGenerator(unsigned int seed) { generator_.seed(seed); }

double NoiseGenerator::generateGaussianNoise(double mean, double stddev) {
  std::normal_distribution<double> distribution(mean, stddev);
  return distribution(generator_);
}

void NoiseGenerator::addGaussianNoise(std::vector<double>& data, double stddev) {
  for (auto& value : data) {
    value += generateGaussianNoise(0.0, stddev);
  }
}

void NoiseGenerator::addGaussianNoise(double& value, double stddev) {
  value += generateGaussianNoise(0.0, stddev);
}

void NoiseGenerator::setSeed(unsigned int seed) { generator_.seed(seed); }

}  // namespace robot_locomotion
