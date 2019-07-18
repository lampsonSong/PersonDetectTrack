#include "ctracker.h"
#include "hungarian/hungarian.h"

#include <thread>

DECLARE_int32(interval);

// ---------------------------------------------------------------------------
// Tracker. Manage tracks. Create, remove, update.
// ---------------------------------------------------------------------------
CTracker::CTracker(std::string tracker_type, size_t max_skipped_frames,
                   size_t max_trace_length, float dist_thres)
    : tracker_type_(tracker_type), max_skipped_frames_(max_skipped_frames),
      max_trace_length_(max_trace_length), next_track_ID_(0),
      dist_thres_(dist_thres) {
  reset();
}

void CTracker::reset() { tracks.clear(); }

void CTracker::update(const cv::Mat &frame, DetectionList &dets) {
  bool b_tracking = true;
  if (tracks.empty()) { // init trackers
    b_tracking = false;
    for (auto det : dets) {
      tracks.push_back(std::make_shared<CTrack>(tracker_type_, det, 0, 300));
      cv::Rect2d currBox;
      currBox.x = det.bbox_.x;
      currBox.y = det.bbox_.y;
      currBox.width = det.bbox_.width;
      currBox.height = det.bbox_.height;
      tracks.back()->init(frame, currBox);
    }
  }

  size_t M = dets.size();
  size_t N = tracks.size();
  assignments_t assignment(N, -1);

  if (!tracks.empty()) { // update trackers
    if (b_tracking) {
      std::vector<std::thread> threads(tracks.size());
      for (size_t t = 0; t < tracks.size(); t++) {
        threads[t] = std::thread(
            [&](std::shared_ptr<CTrack> track) {
              cv::Rect2d currBox;
              bool ret = track->update(frame, currBox);
              track->skippedFrames_++;
              if (ret && currBox.area() > 0) {
                cv::Point2f shift;
                shift.x = track->last_point_.bbox_.x +
                          track->last_point_.bbox_.width * 0.5 - currBox.x -
                          currBox.width * 0.5;
                shift.y = track->last_point_.bbox_.y +
                          track->last_point_.bbox_.height * 0.5 - currBox.y -
                          currBox.height * 0.5;
                for (int si = 0; si < BODY_PART_NUM; si++) {
                  track->last_point_.skeleton_[si] += shift;
                }

                track->confidence_ = 1.0;
                track->last_point_.bbox_.x = currBox.x;
                track->last_point_.bbox_.y = currBox.y;
                track->last_point_.bbox_.width = currBox.width;
                track->last_point_.bbox_.height = currBox.height;

              } else {
                track->confidence_ = 0.0;
              }
            },
            tracks[t]);
      }
      std::for_each(threads.begin(), threads.end(),
                    [](std::thread &x) { x.join(); });

      distMatrix_t Cost(N * M);
      for (size_t i = 0; i < N; i++) {
        for (size_t j = 0; j < M; j++) {
          auto dist = tracks[i]->calc_distance(dets[j]);
          Cost[i + j * N] = dist;
        }
      }

      // Solving assignment problem (tracks and detections)
      AssignmentProblemSolver APS;
      APS.Solve(Cost, N, M, assignment, AssignmentProblemSolver::optimal);

      for (size_t i = 0; i < assignment.size(); i++) {
        if (assignment[i] != -1) {
          if ((1.0 - Cost[i + assignment[i] * N]) < dist_thres_) {
            assignment[i] = -1;
          }
        }
        // If track didn't get detects long time, remove it.
        if (tracks[i]->skippedFrames_ > max_skipped_frames_) {
          tracks.erase(tracks.begin() + i);
          assignment.erase(assignment.begin() + i);
          i--;
        }
      }
    }

    // Search for unassigned detects and start new tracks for them.
    for (size_t i = 0; i < dets.size(); ++i) {
      if (find(assignment.begin(), assignment.end(), i) == assignment.end()) {
        tracks.push_back(
            std::make_unique<CTrack>(tracker_type_, dets[i], 0, 300));
        cv::Rect2d currBox;
        currBox.x = dets[i].bbox_.x;
        currBox.y = dets[i].bbox_.y;
        currBox.width = dets[i].bbox_.width;
        currBox.height = dets[i].bbox_.height;
        tracks.back()->init(frame, currBox);
        tracks.back()->skippedFrames_ = 0;
      }
    }

    // Update trajectory
    for (size_t i = 0; i < tracks.size(); i++) {
      // If track updated less than one time, than filter state is not correct.
      if (i < assignment.size()) {
        int j = assignment[i];
        if (j != -1) {
          // If we have assigned detect, then update using its coordinates,
          tracks[i]->skippedFrames_ = 0;
          tracks[i]->push_back(dets[j]);
          cv::Rect2d currBox;
          currBox.x = dets[j].bbox_.x;
          currBox.y = dets[j].bbox_.y;
          currBox.width = dets[j].bbox_.width;
          currBox.height = dets[j].bbox_.height;
          tracks[i]->init(frame, currBox);
        } else {
          tracks[i]->last_point_.is_raw_ = false;
          tracks[i]->push_back(tracks[i]->last_point_);
        }
      }
    }
  }

  // Remove the unrobust/short-life targets
  for (size_t i = 0; i < tracks.size(); i++) {
    if (tracks[i]->trajectory_.length() == (2 * FLAGS_interval)) {
      if (tracks[i]->is_robust(2 * FLAGS_interval, 0.99 / FLAGS_interval,
                               FLAGS_interval, 0)) {
        next_track_ID_++;
        tracks[i]->set_ID(next_track_ID_);
      } else {
        tracks.erase(tracks.begin() + i);
        i--;
      }
    }
  }
}
