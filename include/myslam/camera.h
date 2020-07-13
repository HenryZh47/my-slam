#ifndef MYSLAM_CAMERA_H
#define MYSLAM_CAMERA_H

/*
 * Camera Class
 * Class to store camera information and transformations
 * Author: Henry Zhang
 */

#include "myslam/common_include.h"

namespace myslam {

class Camera {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using Ptr = std::shared_ptr<Camera>;

  double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;  // camera intrinsics
  double baseline_ = 0;                       // stereo camera baseline
  SE3 pose_;                                  // camera pose
  SE3 pose_inv_;                              // camera pose inverse

  Camera() = default;
  Camera(double fx, double fy, double cx, double cy, double baseline)
      : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline) {}

  ~Camera() = default;

  // get camera pose
  SE3 get_pose() const { return pose_; }

  // get camera intrinsic matrix
  Mat33 K() const {
    Mat33 K;
    K << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
    return K;
  }

  // coordinate transfomations between world frame, camera frame, and image
  Vec3 world2cam(const Vec3 &pw, const SE3 &T_c_w);
  Vec3 cam2world(const Vec3 &pc, const SE3 &T_c_w);
  Vec2 cam2pixel(const Vec3 &pc);
  Vec2 world2pixel(const Vec3 &pw, const SE3 &T_c_w);
  Vec3 pixel2cam(const Vec2 &pp, const double depth = 1);
  Vec3 pixel2world(const Vec2 &pp, const SE3 &T_c_w, const double depth = 1);

};  // class Camera

}  // namespace myslam

#endif  // MYSLAM_CAMERA_H
