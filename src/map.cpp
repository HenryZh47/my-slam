/*
 * Map class
 * Interact with the frontend to insert keyframe and mappoint
 * Interact with the backend to maintain map structure (outlier)
 */

#include "myslam/map.h"
#include "myslam/feature.h"
#include "myslam/mappoint.h"

namespace myslam {

void Map::insert_keyframe(Frame::Ptr frame) {
  // set current frame to this frame
  current_frame_ = frame;

  if (keyframes_.find(frame->keyframe_id_) == keyframes_.end()) {
    // if the frame is not in the keyframe window, add it
    keyframes_.insert(std::make_pair(frame->keyframe_id_, frame));
    active_keyframes_.insert(std::make_pair(frame->keyframe_id_, frame));
  } else {
    // TODO (henryzh47): when would this happen?
    //                   Seeing the same frame twice?
    keyframes_[frame->keyframe_id_] = frame;
    active_keyframes_[frame->keyframe_id_] = frame;
  }

  // maintain a fixed-sized active keyframe window
  if (active_keyframes_.size() > keyframe_window_size) {
    remove_old_keyframe();
  }
}

void Map::insert_map_point(MapPoint::Ptr map_point) {
  if (landmarks_.find(map_point->id_) == landmarks_.end()) {
    landmarks_.insert(std::make_pair(map_point->id_, map_point));
    active_landmarks_.insert(std::make_pair(map_point->id_, map_point));
  } else {
    landmarks_[map_point->id_] = map_point;
    active_landmarks_[map_point->id_] = map_point;
  }
}

void Map::remove_old_keyframe() {
  if (current_frame_ == nullptr) return;
  // find the closest/farthest active keyframe wrt current frame
  double max_dist{0}, min_dist{std::numeric_limits<double>::max()};
  unsigned long max_kf_id, min_kf_id;
  const auto Twc = current_frame_->get_pose().inverse();
  for (const auto &kf : active_keyframes_) {
    // skip if it's current frame
    if (kf.second == current_frame_) continue;
    // get the distance in terms of quaternion norm
    double dist = (kf.second->get_pose() * Twc).log().norm();
    // update min and max dist and id
    if (dist > max_dist) {max_dist = dist; max_kf_id = kf.first;}
    if (dist < min_dist) {min_dist = dist; min_kf_id = kf.first;}
  }

  // threshold for evicting a very close point
  const double min_dist_thresh = 0.2;
  Frame::Ptr frame_to_remove;
  if (min_dist < min_dist_thresh) {
    // delete a very close frame
    frame_to_remove = active_keyframes_.at(min_kf_id);
  } else {
    // delete the farthest frame
    frame_to_remove = active_keyframes_.at(max_kf_id);
  }

  // TODO: put a ROS log here to indicate which frame is getting removed
  // remove keyframe, as well as its associated map point observations
  active_keyframes_.erase(frame_to_remove->keyframe_id_);
  for (const auto feature : frame_to_remove->left_features_) {
    auto mp = feature->map_point_.lock();
    if (mp) { mp->remove_observation(feature); }
  }
  for (const auto feature : frame_to_remove->right_features_) {
    auto mp = feature->map_point_.lock();
    if (mp) { mp->remove_observation(feature); }
  }

  // clean up map with updated observations
  clean_map();
}

void Map::clean_map() {
  unsigned int removed_count = 0;
  for (auto iter = active_landmarks_.begin(); iter != active_landmarks_.end();) {
    if (iter->second->observed_times_ == 0) {
      // this erase function will return the next iterator in map
      iter = active_landmarks_.erase(iter);
      removed_count++;
    } else {
      iter++;
    }
  }
  // TODO: put a ROS log here to show number of points removed
}

Map::KeyframesType Map::get_keyframes() { return keyframes_; }
Map::KeyframesType Map::get_active_keyframes() { return active_keyframes_; }
Map::LandmarksType Map::get_map_points() { return landmarks_; }
Map::LandmarksType Map::get_active_map_points() { return active_landmarks_; }

}  // namespace myslam
