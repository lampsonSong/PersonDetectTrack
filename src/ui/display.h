#pragma once

#include "utils/logging_switch.h"
#include <thread>

#include "io/ring_buffer.h"

class Display {
public:
  Display();
  void init() {
    LOG(INFO) << "Display::init() start";

    LOG(INFO) << "Display::init() stop";
  }

  void start() {
    thread_ = std::make_unique<std::thread>(&Display::run, this);
    LOG(INFO) << "Display::start() start display";
  }
  void stop() {
    b_run_ = false;
    if (thread_) {
      thread_->join();
    }
    LOG(INFO) << "Display::stop() stop display";
  }

private:
  void run();

  volatile bool b_run_;
  RingBuffer *shared_buffer_;

  std::unique_ptr<std::thread> thread_;
};
