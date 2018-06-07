#pragma once

#include <opencv2/core.hpp>
#include <vector>

#include "algorithm/ulspose/types.h"

class FrameData {
public:
  size_t get_uniqueID() const { return global_ID_; }
  void set_uniqueID(size_t ID) { global_ID_ = ID; }

  cv::Mat frame_cap_; // captured frame
  cv::Mat frame_vis_; // frame for displaying
  cv::Mat frame_alg_; // frame for algorithm, size is same with net_resolution

  DetectionList dets;
  TrackingList trks;
  TrackingList short_trks;

  float fps_;
  volatile size_t global_ID_;
};
