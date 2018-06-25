#pragma once

#include <chrono>
#include <ostream>
#include <sstream>
#include <sys/time.h>
#include <time.h>

#include "utils/logging_switch.h"

using namespace std;

using std::chrono::microseconds;
using std::chrono::high_resolution_clock;

class Timer {
public:
  Timer() { reset(); }
  void reset() { _start = high_resolution_clock::now(); }
  float elapsed_ms() const {
    return std::chrono::duration_cast<microseconds>(
               high_resolution_clock::now() - _start)
               .count() /
           1000.f;
  }
  template <typename T, typename Traits>
  friend std::basic_ostream<T, Traits> &
  operator<<(std::basic_ostream<T, Traits> &out, const Timer &timer) {
    return out << timer.elapsed_ms();
  }

private:
  high_resolution_clock::time_point _start;
};

static double get_wall_time() {
  struct timeval tv;
  if (gettimeofday(&tv, NULL)) {
    //  Handle error
    LOG(ERROR) << "gettimeofday(&tv, NULL) failed.";
    return 0;
  }
  return (double)tv.tv_sec + (double)tv.tv_usec * 1e-6;
  // return (double)tv.tv_usec;
}

static bool get_formated_time(std::string &time_str) {
  struct timeval tv;
  struct tm *ptm;
  char time_char_array[64];

  if (gettimeofday(&tv, NULL)) {
    //  Handle error
    LOG(ERROR) << "gettimeofday(&tv, NULL) failed.";
    return false;
  }
  ptm = localtime(&tv.tv_sec);
  strftime(time_char_array, sizeof(time_char_array), "%Y-%m-%d_%H:%M:%S", ptm);

  std::stringstream ss;
  ss << time_char_array;
  ss << ".";
  ss << tv.tv_usec / 1000;
  ss >> time_str;
  return true;
}

static bool get_formated_time_short(std::string &time_str) {
  struct timeval tv;
  struct tm *ptm;
  char time_char_array[64];

  if (gettimeofday(&tv, NULL)) {
    //  Handle error
    LOG(ERROR) << "gettimeofday(&tv, NULL) failed.";
    return false;
  }
  ptm = localtime(&tv.tv_sec);
  strftime(time_char_array, sizeof(time_char_array), "%Y-%m-%d_%H:%M:%S", ptm);

  std::stringstream ss;
  ss << time_char_array;
  ss >> time_str;
  return true;
}
