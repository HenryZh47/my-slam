#ifndef MYSLAM_FRONTEND_H
#define MYSLAM_FRONTEND_H

#include <opencv2/features2d.hpp>

#include "myslam/camera.h"
#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"

namespace myslam {

// forward declaring some
class Backend;

/* front end module status
 * INITING:         Initialize map with first stereo frame
 * TRACKING_GOOD:   Majority of features are being tracked
 * TRACKING_BAD:    Some features are being tracked and find new features
 * LOST:            Lost most of the features, reset
 */
enum class FrontendStatus { INITING, TRACKING_GOOD, TRACKING_BAD, LOST };

class Frontend {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using Ptr = std::shared_ptr<Frontend>;

  Frontend();

  // API, add a frame and compute its pose
  bool add_frame(Frame::Ptr frame);

  void set_map(Map::Ptr map) { map_ = map; }
  void set_backend(std::shared_ptr<Backend> backend) { backend_ = backend; }
  void set_cameras(Camera::Ptr left, Camera::Ptr right) {
    camera_left_ = left;
    camera_right_ = right;
  }

  FrontendStatus get_status() const { return status_; }

 private:
  // track in normal mode
  bool track();

  // reset when lost
  bool reset();

  // track with last frame
  // return num_tracked_points
  int track_last_frame();

  // estimate current frame's pose
  // return number of inliers
  int estimate_current_pose();

  // set current frame as keyframe and insert to backend
  bool insert_keyframe();

  // initialize with stereo images in current frame
  bool stereo_init();

  // detect features in left image in current frame
  // keypoints will be saved in current_frame_
  int detect_features();

  // find the corresponding features in the right image
  // returns the number of features found
  int find_features_in_right();

  // build initial map with single frame
  bool build_init_map();

  // triangulate 2D points to 3D points in current_frame_
  // return num of triangulated points
  int triangulate_new_points();

  // set the features in keyframe as new oberservations of map points
  void set_observations_keyframe();

  Frame::Ptr current_frame_ = nullptr;
  Frame::Ptr last_frame_ = nullptr;
  Camera::Ptr camera_left_ = nullptr;
  Camera::Ptr camera_right_ = nullptr;

  Map::Ptr map_ = nullptr;
  std::shared_ptr<Backend> backend_ = nullptr;

  FrontendStatus status_ = FrontendStatus::INITING;

  SE3 relative_motion_;       // track motion relative to last frame
  int tracking_inliers_ = 0;  // number of tracked inliers, determine new frame

  // params
  // TODO (henryzh47): make these ros params, or in constructor
  int num_features_ = 200;
  int num_features_init_ = 100;
  int num_features_tracking_ = 50;
  int num_features_tracking_bad_ = 20;
  int num_features_needed_for_keyframe_ = 80;

  // feature detector in opencv
  cv::Ptr<cv::GFTTDetector> gftt_;
};

/*
 * Frontend class
 */

}  // namespace myslam

#endif  // MYSLAM_FRONTEND_H
