#ifndef MYSLAM_COMMON_INCLUDE_H
#define MYSLAM_COMMON_INCLUDE_H

/*
 * Common include libraries and type definitions
 * Author: Henry Zhang
 */

// Sophus
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
using SE3 = Sophus::SE3d;
using SO3 = Sophus::SO3d;

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
// double matrices
using MatXX = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using Mat33 = Eigen::Matrix<double, 3, 3>;
using Vec3 = Eigen::Matrix<double, 3, 1>;
using Vec2 = Eigen::Matrix<double, 2, 1>;
// float matrices
using MatXXf = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
using Mat33f = Eigen::Matrix<float, 3, 3>;
using Vec3f = Eigen::Matrix<float, 3, 1>;
using Vec2f = Eigen::Matrix<float, 2, 1>;

#endif  // MYSLAM_COMMON_INCLUDE_H
