#pragma once
#include <array>
#include <deque>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>

#include "algorithm/ulspose/types.h"
#include "utils/buf_queue.h"
#include "utils/logging_switch.h"
#include "utils/timer.h"
#include "utils/utils.h"

DECLARE_double(read_delay);
DECLARE_int32(interval);

class CTrack {
public:
  CTrack(const std::string tracker_type, const BodyBoxPoint &body,
         const size_t trackID, const size_t max_trace_length = 300);
  ~CTrack();

  void init(const cv::Mat &frame, cv::Rect2d &box);
  bool update(const cv::Mat &frame, cv::Rect2d &box);
  void push_back(const BodyBoxPoint &body);
  bool is_robust(int minTraceSize, float minRawRatio, int maxSkippedFrames,
                 size_t base_id = 1);
  void update_height(float h);
  void update_position_speed(const BodyPosition bp);
  void set_ID(const size_t trackID);
  size_t get_ID();
  float calc_distance(const BodyBoxPoint &body);

private:
  size_t get_raw_count(size_t lastPeriod);

public:
  BufQueue<BodyBoxPoint> trajectory_;
  BodyBoxPoint last_point_;

  cv::Point2f speed_;
  cv::Point2f position_;
  float height_;
  double ob_time_;
  float confidence_;
  size_t skippedFrames_;
  size_t max_trace_length_;

  // used for action recognition
  int c_isIntersect_;
  int inter_person_;
  std::deque<cv::Point2f *> violence_pts_;
  std::deque<cv::Point2f *> single_vioPts_;

  bool pre_intersection;
  int provocation_num_;
  int provocation_skip_;
  int cnt_falling_;
  int cnt_falling_skip_;
  int cnt_running_;
  int cnt_running_skip_;
  int cnt_following_;
  int cnt_following_skip_;
  int fight_skip_num;

private:
  BufQueue<float> h_queue_;
  BufQueue<BodyPosition> pos_queue_;
  std::string tracker_type_;
  cv::Ptr<cv::Tracker> tracker_;
  size_t trackID_;
};

typedef std::vector<std::shared_ptr<CTrack>> CTrackList;
