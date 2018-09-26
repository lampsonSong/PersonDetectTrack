#pragma once

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <iostream>
#include <string>
#include <utility> //std::pair
#include <vector>

#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h> // snprintf
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include <gflags/gflags.h>
#include <google/protobuf/text_format.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "algorithm/pose_estimation/core/cvMatToOpInput.hpp"
#include "algorithm/pose_estimation/core/cvMatToOpOutput.hpp"
#include "algorithm/pose_estimation/core/keypointScaler.hpp"
#include "algorithm/pose_estimation/core/opOutputToCvMat.hpp"
#include "algorithm/pose_estimation/core/renderer.hpp"

#include "algorithm/pose_estimation/pose/poseExtractorCaffe.hpp"
#include "algorithm/pose_estimation/pose/poseExtractorCaffe.hpp"
#include "algorithm/pose_estimation/pose/poseParameters.hpp"
#include "algorithm/pose_estimation/pose/poseRenderer.hpp"

#include "algorithm/pose_estimation/utilities/fastMath.hpp"
#include "algorithm/pose_estimation/utilities/keypoint.hpp"

#include "types.h"
#include "ui/display.h"

class PoseEstimation {
public:
  struct PoseParameter {
    int device_id;
    int camera_width;
    int camera_height;
    int net_width;
    int net_height;
    std::istream *proto_pointer;
    std::istream *model_pointer;
  };
  PoseEsimation(PoseParameter param);
  void processFrame(cv::Mat &orig, std::vector<BodyBoxPoint> &person_box_pts);

  std::vector<cv::Point2f> gravity_direct_;

private:
  void confirm_ratio(BodyBoxPoint &person_box_pts);
  bool predict_position(BodyBoxPoint &person_box_pts);
  bool b_set_cam_param_;
  cv::Mat camMatrix_;
  cv::Mat distCoeffs_;
  cv::Vec3d rvec_;
  cv::Vec3d tvec_;

  std::vector<cv::Point3f> point_;
  std::vector<cv::Point2f> imgPoint_;

  pose::Point<int> netInputSize;
  pose::Point<int> netOutputSize;
  pose::Point<int> outputSize;

  pose::Array<float> netInputArray;
  pose::Array<float> outputArray;
  std::vector<float> scaleRatios;

  std::unique_ptr<pose::CvMatToOpInput> cvMatToOpInput;
  std::unique_ptr<pose::CvMatToOpOutput> cvMatToOpOutput;
  std::unique_ptr<pose::OpOutputToCvMat> opOutputToCvMat;

  double scaleInputToOutput;

  std::unique_ptr<pose::PoseExtractorCaffe> poseExtractorCaffe;
  std::unique_ptr<pose::PoseRenderer> poseRenderer;
};
