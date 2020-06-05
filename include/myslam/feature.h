#ifndef MYSLAM_FEATURE_H
#define MYSLAM_FEATURE_H

/*
 * Feature Class
 * Basic structure for slam
 * Class for feature points in the image
 */

#include "myslam/common_include.h"
#include "myslam/frame.h"

namespace myslam {

// forwared decleration of custome types
struct Frame;
struct MapPoint;

/*
 * 2D feature point in image
 * Will associate to a MapPoint after triangulation
 */
struct Feature {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using Ptr = std::shared_ptr<Feature>;

  Feature() = default;
  Feature(Frame::Ptr frame, const cv::KeyPoint &kp)
      : frame_(frame), position_(kp) {}

  std::weak_ptr<Frame> frame_;  // Pointer to the associated frame, use weak_ptr
                                // to avoid circular reference
  std::weak_ptr<MapPoint> map_point_;  // Pointer to associated map point
  cv::KeyPoint position_;              // 2D feature point extracted location

  bool is_outlier_ = false;       // if this feature is outlier
  bool is_on_left_image_ = true;  // if this feature is on the left image

};  // struct Feature

}  // namespace myslam

#endif  // MYSLAM_FEATURE_H
