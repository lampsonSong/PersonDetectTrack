#pragma once
#include <array>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "ctrack.h"

// ----------------------------------------------------------------------
class CTracker {
public:
  CTracker(std::string tracker_type, size_t max_skipped_frames,
           size_t max_trace_length, float dist_thres);

  void update(const cv::Mat &frame, DetectionList &dets);
  void reset();

  CTrackList tracks;

private:
  float dist_thres_;
  size_t max_skipped_frames_;
  size_t max_trace_length_;

  size_t next_track_ID_;
  std::string tracker_type_;
};
