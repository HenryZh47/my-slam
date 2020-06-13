/*
 * Camera Class
 * Class to store camera information and transformations
 * Author: Henry Zhang
 */

#include "myslam/camera.h"

namespace myslam {

Vec3 Camera::world2cam(const Vec3 &pw, const SE3 &T_c_w) {
  return pose_ * T_c_w * pw;
}

Vec3 Camera::cam2world(const Vec3 &pc, const SE3 &T_c_w) {
  return T_c_w.inverse() * pose_inv_ * pc;
}

Vec2 Camera::cam2pixel(const Vec3 &pc) {
  return Vec2(fx_ * pc(0, 0) / pc(2, 0) + cx_, fy_ * pc(1, 0) / pc(2, 0) + cy_);
}

Vec2 Camera::world2pixel(const Vec3 &pw, const SE3 &T_c_w) {
  return cam2pixel(world2cam(pw, T_c_w));
}

Vec3 Camera::pixel2cam(const Vec2 &pp, const double depth) {
  return Vec3((pp(0, 0) - cx_) * depth / fx_, (pp(1, 0) - cy_) * depth / fy_,
              depth);
}

Vec3 Camera::pixel2world(const Vec2 &pp, const SE3 &T_c_w, const double depth) {
  return cam2world(pixel2cam(pp, depth), T_c_w);
}

}  // namespace myslam
