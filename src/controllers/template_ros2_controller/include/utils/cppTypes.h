// MIT License
//
// Copyright (c) 2019 MIT Biomimetic Robotics Lab
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef UTILS__CPPTYPES_H_
#define UTILS__CPPTYPES_H_

#include <eigen3/Eigen/Dense>

// This project uses only the fixed-size Eigen aliases below from the original
// MIT Biomimetic Robotics Lab type utilities.
template <typename T>
using RotMat = Eigen::Matrix<T, 3, 3>;

template <typename T>
using Vec3 = Eigen::Matrix<T, 3, 1>;

template <typename T>
using Mat3 = Eigen::Matrix<T, 3, 3>;

// Quaternion coefficient order is [w, x, y, z].
template <typename T>
using Quat = Eigen::Matrix<T, 4, 1>;

template <typename T>
using Mat4 = Eigen::Matrix<T, 4, 4>;

#endif  // UTILS__CPPTYPES_H_
