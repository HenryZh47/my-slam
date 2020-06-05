/*
 * MapPoint Class
 * Basic structure for slam
 * Maintains a 3D map point in map
 */

#include "myslam/mappoint.h"
#include "myslam/feature.h"

namespace myslam {

MapPoint::MapPoint(unsigned long id, Vec3 pos) : id_(id), pos_(pos) {}

Vec3 MapPoint::get_pos() {
  std::unique_lock<std::mutex> lock(map_point_mutex_);
  return pos_;
}

void MapPoint::set_pos(const Vec3 &pos) {
  std::unique_lock<std::mutex> lock(map_point_mutex_);
  pos_ = pos;
}

void MapPoint::add_observation(Feature::Ptr feature) {
  std::unique_lock<std::mutex> lock(map_point_mutex_);
  observations_.push_back(feature);
  observed_times_++;
}

// TODO (henryzh47): this looks kinda slow, can we do better?
void MapPoint::remove_observation(Feature::Ptr feature) {
  std::unique_lock<std::mutex> lock(map_point_mutex_);
  for (auto itr = observations_.begin(); itr != observations_.end(); itr++) {
    if (itr->lock() == feature) {
      observations_.erase(itr);
      feature->map_point_.reset();
      observed_times_--;
      break;
    }
  }
}

std::list<std::weak_ptr<Feature>> MapPoint::get_observation() {
  std::unique_lock<std::mutex> lock(map_point_mutex_);
  return observations_;
}

MapPoint::Ptr create_map_point() {
  static unsigned long id = 0;
  MapPoint::Ptr new_map_point(new MapPoint);
  new_map_point->id_ = id++;
  return new_map_point;
}

}  // namespace myslam
