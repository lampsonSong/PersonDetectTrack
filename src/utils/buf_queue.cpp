#include "algorithm/ulspose/types.h"
#include "buf_queue.h"
#include <iostream>

template <class T> BufQueue<T>::BufQueue(size_t head_buff_len) {
  head_len_ = head_buff_len;
}

template <class T> T &BufQueue<T>::operator[](size_t i) {
  return dqueue_.at(i);
}

template <class T> T BufQueue<T>::front() {
  if (length() < 1) {
    LOG(ERROR) << "Query Element from Empty Queue";
    return T();
  }
  return dqueue_[0];
}

template <class T> void BufQueue<T>::pop_front() {
  if (length() > 0) {
    dqueue_.erase(dqueue_.begin(), dqueue_.begin() + 1);
  }
}

template <class T> void BufQueue<T>::buff_back(const T &element) {
  dqueue_.push_back(element);
  if (length() > head_len_) {
    dqueue_.erase(dqueue_.begin(), dqueue_.begin() + (length() - head_len_));
  }
}

template <class T> void BufQueue<T>::push_back(const T &element) {
  dqueue_.push_back(element);
}

template <class T> size_t BufQueue<T>::length() const { return dqueue_.size(); }

template <class T> void BufQueue<T>::set_buff_length(size_t len) {
  head_len_ = len;
}

template <class T> bool BufQueue<T>::empty() { return dqueue_.empty(); }

template <class T> void BufQueue<T>::clear() { dqueue_.clear(); }
template class BufQueue<cv::Mat>;
template class BufQueue<float>;
template class BufQueue<double>;
template class BufQueue<int>;
template class BufQueue<BodyBoxPoint>;

template class BufQueue<BodyPosition>;