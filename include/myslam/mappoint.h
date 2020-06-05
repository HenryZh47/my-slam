#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H

/*
 * MapPoint Class
 * Basic structure for slam
 * Maintains a 3D map point in map
 */

#include "myslam/common_include.h"
#include "myslam/feature.h"
#include "myslam/frame.h"

namespace myslam {

// forward declarations of custome types
struct Feature;
struct Frame;

/*
 * MapPooint struct
 * 3D point generated from triangulation of feature points
 */
struct MapPoint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using Ptr = std::shared_ptr<MapPoint>;

  MapPoint() = default;
  MapPoint(unsigned long id, Vec3 pos);
  // get and set map point position (needs to be thread safe)
  Vec3 get_pos();
  void set_pos(const Vec3 &pos);
  // observations to the map point (need to be thread safe)
  void add_observation(Feature::Ptr feature);
  void remove_observation(Feature::Ptr feature);
  std::list<std::weak_ptr<Feature>> get_observation();
  // factory function
  static MapPoint::Ptr create_map_point();

  unsigned long id_ = 0;             // MapPoint ID
  bool is_outlier_ = false;          // map point is outlier
  Vec3 pos_ = Vec3::Zero();          // 3D position of the map point
  std::mutex map_point_mutex_;       // mutex for map point
  unsigned int observed_times_ = 0;  // times the map point being observed
  std::list<std::weak_ptr<Feature>>
      observations_;  // list of features that observes the map point

};  // struct MapPoint

}  // namespace myslam

#endif  // MYSLAM_MAPPOINT_H
