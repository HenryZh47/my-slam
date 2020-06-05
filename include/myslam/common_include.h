#ifndef MYSLAM_COMMON_INCLUDE_H
#define MYSLAM_COMMON_INCLUDE_H

/*
 * Common include libraries and type definitions
 * Author: Henry Zhang
 */

// std
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <typeinfo>
#include <unordered_map>
#include <vector>

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

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
using cv::Mat;

#endif  // MYSLAM_COMMON_INCLUDE_H
