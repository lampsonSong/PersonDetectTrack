#include <chrono>
#include <opencv2/highgui.hpp>

#include "video_reader.h"
#include <iostream>

#include "utils/timer.h"

DECLARE_double(read_delay);
DECLARE_int32(video_start_idx);
extern bool b_run_main;
extern bool b_run_skip;

VideoReader::VideoReader(int dev_id, int w, int h)
    : dev_id_(dev_id), b_run_(true), input_mode_(0) {

  video_cap_ = new cv::VideoCapture(dev_id_);
  if (!video_cap_->isOpened()) {
    LOG(ERROR) << "VideoReader: camera " << dev_id_ << " not found";
    exit(0);
  }
  video_cap_->set(cv::CAP_PROP_FRAME_WIDTH, w);
  video_cap_->set(cv::CAP_PROP_FRAME_HEIGHT, h);
  LOG(INFO) << "set camera " << dev_id_ << " w x h :"
            << "[" << w << " x " << h << "]";

  width_ = video_cap_->get(cv::CAP_PROP_FRAME_WIDTH);
  height_ = video_cap_->get(cv::CAP_PROP_FRAME_HEIGHT);
  if (width_ < 800) {
    height_ = int(800.0 / width_ * height_);
    width_ = 800;
  }
  if (width_ > 1280) {
    height_ = int(1280.0 / width_ * height_);
    width_ = 1280;
  }

  LOG(INFO) << "VideoReader: get camera " << dev_id_;
  LOG(INFO) << "             WIDTH  : " << width_;
  LOG(INFO) << "             HEIGHT : " << height_;

  init();
}

VideoReader::VideoReader(std::string video_path)
    : b_run_(true), input_mode_(1) {

  video_cap_ = new cv::VideoCapture(video_path);
  if (!video_cap_->isOpened()) {
    LOG(ERROR) << "VideoReader: " << video_path << " not found";
    exit(0);
  }
  width_ = video_cap_->get(cv::CAP_PROP_FRAME_WIDTH);
  height_ = video_cap_->get(cv::CAP_PROP_FRAME_HEIGHT);

  if (width_ < 800) {
    height_ = int(800.0 / width_ * height_);
    width_ = 800;
  }
  if (width_ > 1280) {
    height_ = int(1280.0 / width_ * height_);
    width_ = 1280;
  }

  LOG(INFO) << "VideoReader: Video file " << video_path;
  LOG(INFO) << "             WIDTH  : " << width_;
  LOG(INFO) << "             HEIGHT : " << height_;

  video_cap_->set(CV_CAP_PROP_POS_FRAMES, FLAGS_video_start_idx);

  init();
}

VideoReader::VideoReader(std::string img_prefix, int start_idx, int end_idx)
    : img_prefix_(img_prefix), b_run_(true), input_mode_(2),
      start_idx_(start_idx), end_idx_(end_idx) {
  CHECK_LT(start_idx, end_idx) << "Start Index is not less than End Index";

  width_ = 0;
  height_ = 0;
  char name[128];
  int cnt = start_idx_;
  if (cnt <= end_idx_) {
    sprintf(name, "%s%d.png", img_prefix_.c_str(), cnt);
    cv::Mat temp = cv::imread(name);

    if (temp.empty()) {
      LOG(ERROR) << name << " not opened!";
    }
    width_ = temp.cols;
    height_ = temp.rows;

    if (width_ < 800) {
      height_ = int(800.0 / width_ * height_);
      width_ = 800;
    }
    if (width_ > 1280) {
      height_ = int(1280.0 / width_ * height_);
      width_ = 1280;
    }
  }
  init();
}

void VideoReader::init() {
  shared_buffer_ = RingBuffer::instance();

  LOG(INFO) << "VideoReader: successfully initialized";
}

void VideoReader::run() {
  FrameData frame;
  double duration, start;

  start = get_wall_time();
  if (input_mode_ == 0 || input_mode_ == 1) {
    while (b_run_) {
      if (input_mode_ == 0) {
        do {
          video_cap_->read(frame.frame_cap_);
          duration = get_wall_time() - start;
        } while (!b_run_skip && duration < FLAGS_read_delay);
        // cv::medianBlur(frame.frame_cap_, frame.frame_cap_, 3);
        // cv::GaussianBlur(frame.frame_cap_, frame.frame_cap_, cv::Size(3, 3),
        // 0);
      } else {
        video_cap_->read(frame.frame_cap_);
        do {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          duration = get_wall_time() - start;
        } while (!b_run_skip && duration < FLAGS_read_delay);
      }
      if (duration > 100 * FLAGS_read_delay) {
        LOG(ERROR) << "camera does not response in time";
      }
      start = get_wall_time();
      if (!frame.frame_cap_.empty()) {
        if (frame.frame_cap_.size() != getCapSize()) {
          resize(frame.frame_cap_, frame.frame_cap_, getCapSize());
        }

        while (!shared_buffer_->getFrameDataToWrite(frame)) {
          if (b_run_ == false) {
            b_run_main = false;
            return;
          }
        }
      } else {
        std::cout << "[INFO] : reached file end.\n\n";
        b_run_ = false;
      }
    }
  } else if (input_mode_ == 2) {
    char name[128];
    int cnt = start_idx_;
    while (b_run_ && cnt <= end_idx_) {
      duration = 0;
      start = get_wall_time();

      sprintf(name, "%s%d.png", img_prefix_.c_str(), cnt);
      frame.frame_cap_ = cv::imread(name);
      if (frame.frame_cap_.size() != getCapSize()) {
        resize(frame.frame_cap_, frame.frame_cap_, getCapSize());
      }

      while (!shared_buffer_->getFrameDataToWrite(frame)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      while (!b_run_skip && duration < FLAGS_read_delay) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        duration = get_wall_time() - start;
      }
      cnt++;
      if (cnt > end_idx_) {
        cnt = start_idx_;
      }
    }
  } else {
    LOG(ERROR) << "NOT support input mode.";
  }
  b_run_main = false;
  return;
}
