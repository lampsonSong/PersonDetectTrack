#include <chrono>
#include <gflags/gflags.h>
#include <iostream>
#include <string>
#include <thread>

#include <opencv2/imgproc.hpp>

#include "ring_buffer.h"
#include "utils/logging_switch.h"

using namespace std;

DECLARE_int32(ring_buffer_size);

RingBuffer::~RingBuffer() {
  std::vector<FrameData *>::iterator iter;
  for (iter = ring_buffer_.begin(); iter != ring_buffer_.end(); iter++) {
    delete *iter;
    ring_buffer_.erase(iter);
    --iter;
  }
}

RingBuffer::RingBuffer() {
  buffer_size_ = FLAGS_ring_buffer_size;
  global_idx_ = 0;
  head_frame_idx_ = 0;
  tail_frame_idx_ = 0;
  proc_frame_idx_ = 0;

  for (int i = 0; i < buffer_size_; i++) {
    FrameData *new_frame = new FrameData();
    ring_buffer_.push_back(new_frame);
  }

  LOG(INFO) << "RingBuffer: successfully initialized";
  LOG(INFO) << "            BUFFER SIZE: " << buffer_size_;
}

bool RingBuffer::getFrameDataToWrite(const FrameData &frame) {
  if (tail_mtx_.try_lock()) {
    if (head_mtx_.try_lock()) {
      int element_size = (head_frame_idx_ - tail_frame_idx_) % buffer_size_;
      if (element_size < (buffer_size_ - 1)) {
        ring_buffer_[head_frame_idx_]->frame_cap_ = frame.frame_cap_.clone();
        ring_buffer_[head_frame_idx_]->frame_vis_ = // frame.frame_cap_.clone();
            cv::Mat::zeros(frame.frame_cap_.rows, frame.frame_cap_.cols,
                           frame.frame_cap_.type());
        cv::addWeighted(ring_buffer_[head_frame_idx_]->frame_vis_, 0.875,
                        frame.frame_cap_, 0.125, 0,
                        ring_buffer_[head_frame_idx_]->frame_vis_);

        cv::resize(ring_buffer_[head_frame_idx_]->frame_cap_,
                   ring_buffer_[head_frame_idx_]->frame_alg_, alg_size_);

        ring_buffer_[head_frame_idx_]->set_uniqueID(global_idx_);
        global_idx_++;
        head_frame_idx_++;
        head_frame_idx_ = head_frame_idx_ % buffer_size_;
        tail_mtx_.unlock();
        head_mtx_.unlock();
        LOG(INFO) << "VideoReader read/grab frame " << global_idx_ - 1;
        return true;
      }
      head_mtx_.unlock();
    }
    tail_mtx_.unlock();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  return false;
}

bool RingBuffer::getFrameDataToShow(FrameData *&frame, int &idx) {
  if (proc_mtx_.try_lock()) {
    if (tail_mtx_.try_lock()) {
      if (tail_frame_idx_ != proc_frame_idx_) {
        idx = tail_frame_idx_;
        frame = ring_buffer_[tail_frame_idx_];
        tail_frame_idx_++;
        tail_frame_idx_ = tail_frame_idx_ % buffer_size_;
        proc_mtx_.unlock();
        tail_mtx_.unlock();
        return true;
      }
      tail_mtx_.unlock();
    }
    proc_mtx_.unlock();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  return false;
}

bool RingBuffer::getFrameDataToProcess(FrameData *&frame) {
  if (proc_mtx_.try_lock()) {
    // try to lock, prevent from dead lock with other thread
    if (head_mtx_.try_lock()) {
      // if not empty, get the frame
      if (proc_frame_idx_ != head_frame_idx_) {
        frame = ring_buffer_[proc_frame_idx_];
        head_mtx_.unlock();
        proc_mtx_.unlock();
        return true;
      }
      head_mtx_.unlock();
    }
    proc_mtx_.unlock();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  return false;
}

void RingBuffer::doneProcess() {
  proc_mtx_.lock();
  proc_frame_idx_++;
  proc_frame_idx_ = proc_frame_idx_ % buffer_size_;
  proc_mtx_.unlock();
}

RingBuffer *RingBuffer::instance() {
  static RingBuffer *inst = NULL;
  if (inst == NULL) {
    inst = new RingBuffer();
  }
  return inst;
}

void RingBuffer::skipAllFrame() {
  proc_mtx_.lock();
  head_mtx_.lock();
  proc_frame_idx_ = head_frame_idx_;
  proc_mtx_.unlock();
  head_mtx_.unlock();
}

void RingBuffer::setFrameAlgSize(const int w, const int h) {
  alg_size_ = cv::Size(w, h);
}