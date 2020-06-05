#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

/*
 * Frame Class
 * Basic structure for slam
 * Maintains one frame of image and its associated features
 */

#include "myslam/camera.h"
#include "myslam/common_include.h"

namespace myslam {

// forward declaration of custome types
struct MapPoint;
struct Feature;

/*
 * Frame Class
 * Assign Frame ID for each of the frame
 * Assign Key Frame ID for each of the key frame
 */
struct Frame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using Ptr = std::shared_ptr<Frame>;

  // constructors
  Frame() = default;
  Frame(unsigned long id, double time_stamp, const SE3 &pose,
        const Mat &left_img, const Mat &right_img);
  // set and get frame pose (thread safe)
  SE3 get_pose();
  void set_pose(const SE3 pose);
  // set keyframe and assign keyframe id
  void set_keyframe();
  // factory constructor
  static Frame::Ptr create_frame();

  unsigned long id_ = 0;           // ID of this frame
  unsigned long keyframe_id_ = 0;  // key frame ID of this frame
  bool is_keyframe_ = false;       // if the frame is a key frame
  SE3 pose_;                       // pose of the frame (Tcw)
  std::mutex pose_mutex_;          // lock for frame pose
  cv::Mat left_img_, right_img_;   // stereo images of the frame

  // extracted features of left and right images
  std::vector<std::shared_ptr<Feature>> left_features_;
  std::vector<std::shared_ptr<Feature>> right_features;

  double time_stamp_{0.0};  // timestamp of the frame (currently not used)

};  // struct Frame
}  // namespace myslam

#endif  // MYSLAM_FRAME_H
