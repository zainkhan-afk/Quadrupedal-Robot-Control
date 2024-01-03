#ifndef TYPES_H
#define TYPES_H

#include <vector>
#include <eigen3/Eigen/Dense>


// Vectors
template <typename T>
using Vec2 = typename Eigen::Matrix<T, 2, 1>;

template <typename T>
using Vec3 = typename Eigen::Matrix<T, 3, 1>;

template <typename T>
using Vec4 = typename Eigen::Matrix<T, 4, 1>;

template <typename T>
using Vec6 = typename Eigen::Matrix<T, 6, 1>;

template <typename T>
using Vec12 = typename Eigen::Matrix<T, 12, 1>;

template <typename T>
using Vec18 = typename Eigen::Matrix<T, 18, 1>;

// Special Purpose Vectors
template <typename T>
using VecSp = typename Eigen::Matrix<T, 6, 1>;

template <typename T>
using Quat = typename Eigen::Matrix<T, 4, 1>;

// Matrices
template <typename T>
using Mat3 = typename Eigen::Matrix<T, 3, 3>;

template <typename T>
using MatSp = typename Eigen::Matrix<T, 6, 6>;

#endif