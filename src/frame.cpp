/*
 * Frame Class
 * Basic structure for slam
 * Maintains one frame of image and its associated features
 */

#include "myslam/frame.h"

namespace myslam {

Frame::Frame(unsigned long id, double time_stamp, const SE3 &pose,
             const Mat &left_img, const Mat &right_img)
    : id_(id),
      time_stamp_(time_stamp),
      pose_(pose),
      left_img_(left_img),
      right_img_(right_img) {}

SE3 Frame::get_pose() {
  std::unique_lock<std::mutex> lock(pose_mutex_);
  return pose_;
}

void Frame::set_pose(const SE3 pose) {
  std::unique_lock<std::mutex> lock(pose_mutex_);
  pose_ = pose;
}

void Frame::set_keyframe() {
  static unsigned long factory_keyframe_id = 0;
  is_keyframe_ = true;
  keyframe_id_ = factory_keyframe_id++;
}

Frame::Ptr create_frame() {
  static unsigned long factory_frame_id = 0;
  Frame::Ptr new_frame(new Frame);
  new_frame->id_ = factory_frame_id++;
  return new_frame;
}

}  // namespace myslam
