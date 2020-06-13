#ifndef MYSLAM_MAP_H
#define MYSLAM_MAP_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"

namespace myslam {

/*
 * Map class
 * Interact with the frontend to insert keyframe and mappoint
 * Interact with the backend to maintain map structure (outlier)
 */

class Map {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  using Ptr = std::shared_ptr<Map>;
  using LandmarksType = std::unordered_map<unsigned long, MapPoint::Ptr>;
  using KeyframesType = std::unordered_map<unsigned long, Frame::Ptr>;

  Map() = default;

  // add a keyframe
  void insert_keyframe(Frame::Ptr frame);
  // add a mappoint to landmark
  void insert_map_point(MapPoint::Ptr map_point);

  // get all the keyframes
  KeyframesType get_keyframes();
  // get all the mappoints
  LandmarksType get_map_points();

  // get all active keyframes
  KeyframesType get_active_keyframes();
  // get all active map points
  LandmarksType get_active_map_points();

  // clear map (get rid of map points with zero observations)
  void clean_map();

 private:
  // maintain a sliding window of active key frames
  void remove_old_keyframe();

  std::mutex data_mutex_;
  LandmarksType landmarks_;
  LandmarksType active_landmarks_;
  KeyframesType keyframes_;
  KeyframesType active_keyframes_;

  Frame::Ptr current_frame_ = nullptr;

  // TODO (henryzh47): maybe make this a ROS parameter?
  const int keyframe_window_size = 7;

};  // class Map

}  // namespace myslam

#endif  // MYSLAM_MAP_H
