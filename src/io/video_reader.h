#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include <thread>

#include "utils/logging_switch.h"

#include "ring_buffer.h"

class VideoReader {
public:
  VideoReader(int dev_id, int w = 1280, int h = 720);
  VideoReader(std::string video_path);
  VideoReader(std::string img_prefix, int start_idx, int end_idx);

  void start() {
    thread_ = std::make_unique<std::thread>(&VideoReader::run, this);
    LOG(INFO) << "VideoReader::start() start video capture";
  }
  void stop() {
    b_run_ = false;
    if (thread_ != NULL) {
      thread_->join();
    }
    LOG(INFO) << "VideoReader::stop() stop video capture";
  }

  cv::Size getCapSize() { return cv::Size(width_, height_); }

private:
  void init();
  void run();

  int width_;
  int height_;
  volatile bool b_run_;

  int dev_id_;
  cv::VideoCapture *video_cap_;

  std::string img_prefix_;
  int start_idx_, end_idx_;

  int input_mode_;
  RingBuffer *shared_buffer_;
  std::unique_ptr<std::thread> thread_;
};
