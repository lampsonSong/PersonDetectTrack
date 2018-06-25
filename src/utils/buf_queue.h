#pragma once
#include <deque>
#include <memory>
#include <opencv2/core.hpp>

#include "utils/logging_switch.h"

template <class T> class BufQueue {
public:
  BufQueue(size_t head_buff_len = 60);

  T &operator[](size_t i);
  T front();
  void pop_front();
  void push_back(const T &element);
  void buff_back(const T &element);

  void set_buff_length(size_t len);
  size_t length() const;

  bool empty();
  void clear();

private:
  std::deque<T> dqueue_;
  size_t head_len_;
};
