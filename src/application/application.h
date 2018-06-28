#include "algorithm/tracker/ctracker.h"
#include "algorithm/pose_estimation/pose.h"
#include "io/ring_buffer.h"
#include "ui/display.h"

#include <deque>

class VideoAnalysis {
public:
  void init(const cv::Size size, int net_width, int net_height,
            Display &display);
  void run();

private:
  std::vector<cv::Scalar> colors_;

  RingBuffer *shared_buffer_;
  std::unique_ptr<CTracker> tracker_;
  std::unique_ptr<PoseEstimation> pose_;

  double staying_time_;

  float marker_length_;
  int makrer_dict_id_;

  float w_scale_;
  float h_scale_;

  int tracking_time_;
  int tracking_size_;
  std::deque<cv::Mat> tracking_buffer_mats_;
  std::deque<DetectionList> tracking_buffer_dets_;
};
