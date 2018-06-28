#include <chrono>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>

#include "bank_video_analysis.h"
#include "utils/logging_switch.h"
#include "utils/timer.h"
#include "utils/utils.h"

extern bool b_run_main;
bool b_run_skip = false;
bool b_init_finished = false;

DECLARE_int32(gpu_id);
DECLARE_int32(net_size);
DECLARE_int32(interval);
DECLARE_int32(camera_id);
DECLARE_int32(model_mode);
DECLARE_string(conf_json);
DECLARE_string(param_json);
DECLARE_string(video_path);
DECLARE_string(caffemodel);
DECLARE_string(caffeproto);
DECLARE_double(read_delay);

template <typename CharT, typename TraitsT = std::char_traits<CharT>>
class vectorwrapbuf : public std::basic_streambuf<CharT, TraitsT> {
public:
  vectorwrapbuf(std::vector<CharT> &vec) {
    std::streambuf::setg(vec.data(), vec.data(), vec.data() + vec.size());
  }
};

void VideoAnalysis::init(const cv::Size size, int net_width, int net_height,
                             Display &display) {
  colors_.push_back(cv::Scalar(255, 0, 0));
  colors_.push_back(cv::Scalar(0, 255, 0));
  colors_.push_back(cv::Scalar(0, 0, 255));
  colors_.push_back(cv::Scalar(255, 255, 0));
  colors_.push_back(cv::Scalar(0, 255, 255));
  colors_.push_back(cv::Scalar(255, 0, 255));
  colors_.push_back(cv::Scalar(128, 255, 0));
  colors_.push_back(cv::Scalar(0, 128, 255));
  colors_.push_back(cv::Scalar(255, 0, 128));
  colors_.push_back(cv::Scalar(255, 128, 0));
  colors_.push_back(cv::Scalar(0, 255, 128));
  colors_.push_back(cv::Scalar(128, 0, 255));

  LOG(INFO) << "VideoAnalysis::init() start";
  tracker_ = std::make_unique<CTracker>(
      "KCF", // tracker type: MIL,BOOSTING,KCF,TLD,MEDIANFLOW,GOTURN
      30,    // max skipped frame number
      60,    // max trace lenth
      0.05   // distance threshold
      );
  tracking_time_ = 0;
  tracking_size_ = 0;

  shared_buffer_ = RingBuffer::instance();

  w_scale_ = static_cast<float>(size.width) / net_width;
  h_scale_ = static_cast<float>(size.height) / net_height;

  PoseEstimation::PoseParameter param = {FLAGS_gpu_id,        // gpu device id
                                     net_width,           // camera width
                                     net_height,          // camera height
                                     net_width,           // net width
                                     net_height,          // net height
                                     NULL,         NULL}; // caffe model file

  // load the encrypted model
  std::vector<string> fileLists;
  string proto_file;
  string model_file;
  string model_bin;
  if (FLAGS_model_mode == 0) {
    proto_file = "pose_deploy_cpm.prototxt";
    model_file = "cpm.caffemodel";
    model_bin = "data/model/model_small.bin";
  } else {
    proto_file = "pose_deploy_paf.prototxt";
    model_file = "paf.caffemodel";
    model_bin = "data/model/model_big.bin";
  }
  fileLists.push_back(proto_file);
  fileLists.push_back(model_file);
  ULSeeEncryption ule(fileLists);
  auto decoderFiles = ule.doDecoder(model_bin);

  std::vector<char> proto_name = decoderFiles[proto_file];
  std::vector<char> model_name = decoderFiles[model_file];

  vectorwrapbuf<char> protobuf(proto_name);
  std::istream proto_stream(&protobuf);
  param.proto_pointer = &proto_stream;

  vectorwrapbuf<char> modelbuf(model_name);
  std::istream model_stream(&modelbuf);
  param.model_pointer = &model_stream;

  pose_ = std::make_unique<PoseEstimation>(param);


  FrameData *frame;
  do {
    if (shared_buffer_->getFrameDataToProcess(frame)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));

      shared_buffer_->doneProcess();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  } while (b_run_main && !b_init_finished);



  shared_buffer_->skipAllFrame();

  display.init();
  
  LOG(INFO) << "Pose model input size : " << net_width << " x "
            << net_height;
  LOG(INFO) << "        model output size: " << net_width << " x "
            << net_height;
  LOG(INFO) << "VideoAnalysis::init() finish";
}

void VideoAnalysis::run() {
  FrameData *frame;
  int frame_width;
  double time_a, time_b, sum = 0;

  time_a = get_wall_time();
  while (b_run_main) {
    if (shared_buffer_->getFrameDataToProcess(frame)) {
      LOG(INFO) << "VideoAnalysis start processing frame "
                << frame->get_uniqueID();
      frame_width = frame->frame_cap_.cols;
      time_a = get_wall_time();
      frame->dets.clear();
      frame->trks.clear();
      if (!b_run_skip) {
        if ((frame->get_uniqueID() % FLAGS_interval) == 0) {
          pose_->processFrame(frame->frame_alg_, frame->dets);
        }

        tracker_->update(frame->frame_alg_, frame->dets);

        if ((frame->get_uniqueID() % FLAGS_interval) == 0) {
          for (auto &det : frame->dets) {
            det.bbox_.x *= w_scale_;
            det.bbox_.width *= w_scale_;
            det.bbox_.y *= h_scale_;
            det.bbox_.height *= h_scale_;

            for (int si = 0; si < BODY_PART_NUM; si++) {
              det.skeleton_[si].x *= w_scale_;
              det.skeleton_[si].y *= h_scale_;
            }

            det.pix_shoulder_.x *= w_scale_;
            det.pix_shoulder_.y *= h_scale_;
            det.pix_hip_.x *= w_scale_;
            det.pix_hip_.y *= h_scale_;
            det.pix_knee_.x *= w_scale_;
            det.pix_knee_.y *= h_scale_;
            det.pix_foot_.x *= w_scale_;
            det.pix_foot_.y *= h_scale_;
          }
        }
        for (auto &trk : tracker_->tracks) {
          auto last_track = trk->last_point_;
          IDbodyBoxPoint scale_res;

          last_track.bbox_.x *= w_scale_;
          last_track.bbox_.width *= w_scale_;
          last_track.bbox_.y *= h_scale_;
          last_track.bbox_.height *= h_scale_;
          scale_res.bbox_ = last_track.bbox_;

          for (int si = 0; si < BODY_PART_NUM; si++) {
            last_track.skeleton_[si].x *= w_scale_;
            last_track.skeleton_[si].y *= h_scale_;
            scale_res.skeleton_[si] = last_track.skeleton_[si];
          }

          last_track.pix_shoulder_.x *= w_scale_;
          last_track.pix_shoulder_.y *= h_scale_;

          last_track.pix_hip_.x *= w_scale_;
          last_track.pix_hip_.y *= h_scale_;

          last_track.pix_knee_.x *= w_scale_;
          last_track.pix_knee_.y *= h_scale_;

          last_track.pix_foot_.x *= w_scale_;
          last_track.pix_foot_.y *= h_scale_;

          scale_res.pix_shoulder_ = last_track.pix_shoulder_;
          scale_res.pix_hip_ = last_track.pix_hip_;
          scale_res.pix_knee_ = last_track.pix_knee_;
          scale_res.pix_foot_ = last_track.pix_foot_;

          scale_res.is_robust_ = trk->is_robust(10, 0.2, 50);
          scale_res.trackID_ = trk->get_ID();
          scale_res.body_state_ = 0;

          frame->trks.push_back(scale_res);

          if (last_track.is_raw_) {
            BodyPosition bpos;
            cv::Point3f people_xyz;
            if (last_track.pix_foot_.x > 0 && last_track.pix_foot_.y > 0) {
              float shoulder_height;
              people_xyz = {0, 0, 0};
              
              // by foot or by shoulder
              bpos.body_position_ = cv::Point2f(people_xyz.x, people_xyz.y);
              bpos.is_valid_ = true;
            } else {
              // pop up height
              if (trk->height_ > 0.1) {
                people_xyz = {0, 0, trk->height_};
                
				bpos.body_position_ = cv::Point2f(people_xyz.x, people_xyz.y);
                bpos.is_valid_ = true;
              }
            }
			if(trk->skippedFrames_ < 10)
            	trk->update_position_speed(bpos);
          }
        }


      }else{
        tracker_->reset();
      }


      time_b = get_wall_time();
      if ((time_b - time_a) > 0.006) {
        sum = 0.99 * sum + 0.01 / (time_b - time_a);
        frame->fps_ = sum;
      }
      shared_buffer_->doneProcess();

      LOG(INFO) << "VideoAnalysis finish processing frame "
                << frame->get_uniqueID();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
