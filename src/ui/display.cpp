#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>

#include "algorithm/pose_estimation/pose/poseParameters.hpp"
#include "display.h"
#include "io/frame_data.h"
#include "utils/timer.h"

extern bool b_run_main;
extern bool b_run_skip;
extern bool b_init_finished;

DECLARE_int32(interval);
DECLARE_int32(flag_record);
DECLARE_double(read_delay);

Display::Display() {
  
  b_run_ = true;
  b_init_finished = true;
  
  shared_buffer_ = RingBuffer::instance();
}

void Display::run() {
  int read_fps = (int)(1.0 / FLAGS_read_delay);
  int win_width;
  int win_height;
  int radius = 16;
  int thicknessCircle = 6;
  int lineType = 8;
  char message[128];
  FrameData *frame;
  int key = 0;

  std::string time_str;
  cv::Mat tips;


  while (b_run_) {
    int rb_idx;
    if (shared_buffer_->getFrameDataToShow(frame, rb_idx)) {
      LOG(INFO) << "Display::run() start on frame: " << frame->get_uniqueID();
      win_width = frame->frame_cap_.cols;
      win_height = frame->frame_cap_.rows;

      cv::Size temp_size = frame->frame_cap_.size();
      // cv::Size temp_size = frame->frame_cap_.size() / 2;
      temp_size.width = temp_size.width & 0xfffA;
      temp_size.height = temp_size.height & 0xfffA;

        // tracking Window
        cv::Mat trackwindow = frame->frame_cap_.clone();
        for (int person = 0; person < frame->trks.size(); person++) {
          if (frame->trks[person].is_robust_) {
            cv::rectangle(trackwindow, frame->trks[person].bbox_,
                          CV_RGB(0x87, 0xce, 0xff), 2, 8);
            sprintf(message, "ID: %lu", frame->trks[person].trackID_);
            cv::putText(trackwindow, message,
                        cv::Point(frame->trks[person].bbox_.x,
                                  frame->trks[person].bbox_.y - 10),
                        CV_FONT_HERSHEY_PLAIN, 2, CV_RGB(200, 255, 0), 4);
          }
        }

        tips = trackwindow.clone();
        cv::rectangle(tips, cv::Rect(0, 0, 800, 40), CV_RGB(128, 128, 255),
                      CV_FILLED);
        cv::addWeighted(tips, 0.6, trackwindow, 0.4, 0, trackwindow);
        get_formated_time_short(time_str);
        sprintf(message, "%20s, %3.1f fps, %2lu people", time_str.c_str(),
                frame->fps_, frame->trks.size());
        LOG(INFO) << message;
        cv::putText(trackwindow, message, cv::Point(20, 30),
                    CV_FONT_HERSHEY_PLAIN, 2, CV_RGB(200, 255, 0), 2);

    
      

        cv::namedWindow("PersonDetectAndTracking",
                        cv::WINDOW_NORMAL);
   
        cv::imshow("PersonDetectAndTracking",
                   trackwindow);

      LOG(INFO) << "Display::run() stop on frame: " << frame->get_uniqueID();
    }
    key = cv::waitKey(1);
    if ((0xff & key) == 's') {
      b_run_skip = !b_run_skip;
    }

    if ((key & 0x7f) == 'q') {
      b_init_finished = true;
      cv::destroyAllWindows();
      b_run_ = false;
      LOG(INFO) << "Display : press 'q' to quit";
    }
  }
  b_run_main = false;
}
