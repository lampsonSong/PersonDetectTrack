#include "ctrack.h"

CTrack::CTrack(const std::string tracker_type, const BodyBoxPoint &body,
               const size_t trackID, const size_t max_trace_length)
    : tracker_type_(tracker_type), trackID_(trackID), skippedFrames_(0),
      max_trace_length_(max_trace_length), confidence_(1.0f) {
  h_queue_.set_buff_length(6);
  pos_queue_.set_buff_length(FLAGS_interval); // raw frame numbers
  trajectory_.set_buff_length(max_trace_length_);
  trajectory_.buff_back(body);
  last_point_ = body;
  height_ = -1.0;
  speed_ = cv::Point2f(0, 0);
  position_ = cv::Point2f(-100.f, -100.f);
  ob_time_ = get_wall_time();

  cnt_falling_ = 0;
  cnt_falling_skip_ = 0;
  cnt_following_ = 0;
  cnt_following_skip_ = 0;
  cnt_running_ = 0;
  cnt_running_skip_ = 0;

  provocation_num_ = 0;
  provocation_skip_ = 0;

  fight_skip_num = 0;

  pre_intersection = false;

  LOG(INFO) << "new tracking Id: " << trackID_;
}
CTrack::~CTrack() {
  if (!tracker_.empty()) {
    tracker_.release();
  }
}

float CTrack::calc_distance(const BodyBoxPoint &body) {
  return 1.0 - rectOverlap_enhanced(body.bbox_, last_point_.bbox_);
}

void CTrack::init(const cv::Mat &frame, cv::Rect2d &box) {
  if (!tracker_.empty()) {
    tracker_.release();
  }
  if (tracker_type_.compare("KCF") == 0) {
    cv::TrackerKCF::Params param;
    param.desc_pca = cv::TrackerKCF::GRAY | cv::TrackerKCF::CN;
    param.desc_npca = 0;
    param.compress_feature = true;
    param.compressed_size = 2;
    param.resize = true;
    tracker_ = cv::TrackerKCF::createTracker(param);
  } else {
    tracker_ = cv::Tracker::create(tracker_type_);
  }
  if (tracker_.empty()) {
    LOG(ERROR) << "tracker_ in id(" << trackID_ << ") is empty.";
  }
  tracker_->init(frame, box);
}

bool CTrack::update(const cv::Mat &frame, cv::Rect2d &box) {
  bool ret = tracker_->update(frame, box);
  box = box & cv::Rect2d(0, 0, frame.cols, frame.rows);
  return ret;
}

void CTrack::push_back(const BodyBoxPoint &body) {
  trajectory_.buff_back(body);
  last_point_ = body;
}

size_t CTrack::get_raw_count(size_t lastPeriod) {
  size_t res = 0;

  size_t i = 0;
  if (lastPeriod < trajectory_.length()) {
    i = trajectory_.length() - lastPeriod;
  }
  for (; i < trajectory_.length(); ++i) {
    if (trajectory_[i].is_raw_) {
      ++res;
    }
  }

  return res;
}

bool CTrack::is_robust(int minTraceSize, float minRawRatio,
                       int maxSkippedFrames, size_t base_id) {
  bool res = trajectory_.length() >= static_cast<size_t>(minTraceSize);
  res &= get_raw_count(trajectory_.length()) >=
         (minRawRatio * trajectory_.length());
  res &= maxSkippedFrames > skippedFrames_;
  res &= confidence_ > 0.5;
  res &= (trackID_ >= base_id);
  return res;
}

void CTrack::update_height(float h) {
  if (h > 0)
    h_queue_.buff_back(h);
  if (h_queue_.length() > 3) {
    float sum = 0;
    for (size_t i = 0; i < h_queue_.length(); ++i) {
      sum += h_queue_[i];
    }
    float mean = sum / h_queue_.length();

    sum = 0.0;
    for (size_t i = 0; i < h_queue_.length(); ++i) {
      sum += (h_queue_[i] - mean) * (h_queue_[i] - mean);
    }
    float stdev = sqrt(sum / (h_queue_.length() - 1));

    sum = 0;
    int num = 0;
    for (size_t i = 0; i < h_queue_.length(); ++i) {
      if (fabs(h_queue_[i] - mean) <= (stdev + 1e-6)) {
        sum += h_queue_[i];
        num++;
      }
    }
    if (num > 0)
      height_ = sum / num;
  }
}

void CTrack::update_position_speed(const BodyPosition bp) {
  pos_queue_.buff_back(bp);
  position_ = cv::Point2f(-100.f, -100.f);
  if (pos_queue_.length() > 1) {
    if (pos_queue_[pos_queue_.length() - 1].is_valid_) {
      position_ = pos_queue_[pos_queue_.length() - 1].body_position_;
    }
  }

  speed_ = cv::Point2f(0, 0);
  if (pos_queue_.length() > 1) {
    if (!pos_queue_[pos_queue_.length() - 1].is_valid_)
      return;
    int num = 0;
    cv::Point2f sum = cv::Point2f(0, 0);
    for (size_t i = 1; i < pos_queue_.length(); ++i) {
      if (!pos_queue_[i].is_valid_) {
        continue;
      }
      int j;
      for (j = i - 1; j >= 0; j--) {
        if (pos_queue_[j].is_valid_) {
          break;
        }
      }
      if (j < 0) {
        continue;
      }

      cv::Point2f temp =
          pos_queue_[i].body_position_ - pos_queue_[j].body_position_;
      float maxd =
          distance(pos_queue_[i].body_position_, pos_queue_[j].body_position_) /
          float(i - j);
      if (maxd < (0.4 * FLAGS_interval)) {
        // (0.4 * FLAGS_interval) is the
        // distance between every key pose frame
        // 36 kmph / 3.6mph /25fps = 0.4
        // must be less than 36 Km/h speed
        sum += float(i) * temp / float(i - j);
        num += i;
      }
    }
    if (num > 0) {
      speed_ = sum / num;
      speed_ = speed_ / FLAGS_read_delay / FLAGS_interval;
    }
  }
}

void CTrack::set_ID(const size_t trackID) { trackID_ = trackID; }

size_t CTrack::get_ID() { return trackID_; }
