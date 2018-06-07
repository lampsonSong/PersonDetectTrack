#pragma once
#include "frame_data.h"
#include <opencv2/core.hpp>
#include <vector>
#include <mutex>

class RingBuffer {
public:
  ~RingBuffer();

  static RingBuffer *instance();

  bool getFrameDataToWrite(const FrameData &frame);
  bool getFrameDataToProcess(FrameData *&frame);
  bool getFrameDataToShow(FrameData *&frame, int &idx);
  void setFrameAlgSize(const int w, const int h);
  void doneProcess();
  void skipAllFrame();

private:
  RingBuffer();

  int buffer_size_;
  cv::Size alg_size_;

  unsigned long long global_idx_;
  volatile int head_frame_idx_;
  volatile int tail_frame_idx_;
  volatile int proc_frame_idx_;

  std::vector<FrameData *> ring_buffer_;

  std::mutex head_mtx_;
  std::mutex tail_mtx_;
  std::mutex proc_mtx_;
};
