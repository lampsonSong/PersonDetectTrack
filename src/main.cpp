#include <chrono>
#include <gflags/gflags.h>
#include <iostream>
#include <signal.h>
#include <string>
#include <thread>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>

#include "application/application.h"
#include "io/ring_buffer.h"
#include "io/video_reader.h"
#include "ui/display.h"

DEFINE_int32(net_size, 256, "Multiples of 8.");
DEFINE_int32(gpu_id, 0, "gpu device id.");
DEFINE_int32(model_mode, 1, "model mode.");
DEFINE_int32(ring_buffer_size, 30, "ring bufer length");
DEFINE_int32(interval, 4, "detecting frame interval");
DEFINE_double(read_delay, 0.040, "software reading delay.");

DEFINE_string(img_prefix, "", "input image sequence path prefix.");
DEFINE_int32(img_start_idx, 0, "input image sequence start index.");
DEFINE_int32(img_stop_idx, 0, "input image sequence stop index.");

DEFINE_string(video_path, "", "input video file path.");
DEFINE_int32(video_start_idx, 0, "start index in video capture .");

DEFINE_int32(camera_id, 0, "camera id.");
DEFINE_int32(flag_record, 0, "0 for raw video, 1 for event pieces, \
    due to limited HW de/encode engine, one to record the UI, one  \
    to record raw video or event pieces");

DECLARE_bool(version);
DECLARE_bool(help);
DECLARE_bool(helpfull);
DECLARE_bool(helpxml);
DECLARE_bool(helpshort);

// main loop flag
int ret = 0;
bool b_run_main = true;
void sigHandle(int sig);

int main(int argc, char **argv) {
  // signal handler
  signal(SIGINT, sigHandle);

  // glog initializing
  google::InitGoogleLogging(argv[0]);
  google::SetLogDestination(google::GLOG_INFO, "./log/");
  google::InstallFailureSignalHandler();

  FLAGS_colorlogtostderr = true;
  FLAGS_alsologtostderr = 1;
  FLAGS_max_log_size = 10;

  // gflags initializing
  std::string usage("Person detect and tracking. \nUsage:\n");
  usage += std::string(argv[0]) + "\n";
  usage += std::string(argv[0]) + " --camera_id=0\n";

  gflags::SetUsageMessage(usage);
  gflags::SetVersionString("0.1.0.0");
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_help || FLAGS_helpxml || FLAGS_helpfull || FLAGS_helpshort) {
    FLAGS_help = false;
    FLAGS_helpfull = false;
    FLAGS_helpxml = false;
    FLAGS_helpshort = true;
    gflags::HandleCommandLineHelpFlags();
  }

  boost::filesystem::path dir("log");
  if (!boost::filesystem::is_directory(dir) &&
      !boost::filesystem::create_directory(dir)) {
    LOG(ERROR) << "Could not write to or create directory " << dir;
    return 1;
  }

  // RingBuffer initialization
  RingBuffer *rb = RingBuffer::instance();

  std::unique_ptr<VideoReader> vreader;
  if (!FLAGS_img_prefix.empty()) {
    vreader = std::make_unique<VideoReader>(
        FLAGS_img_prefix, FLAGS_img_start_idx, FLAGS_img_stop_idx);
  } else if (!FLAGS_video_path.empty()) {
    vreader = std::make_unique<VideoReader>(FLAGS_video_path);
  } else {
    vreader = std::make_unique<VideoReader>(FLAGS_camera_id, 1280, 720);
  }
  vreader->start();
  cv::Size size = vreader->getCapSize();
  int net_width, net_height;
  net_height = FLAGS_net_size;
  net_width = (size.width * net_height + (size.height >> 1)) / size.height;
  net_width = (net_width + 0x7) & 0xFFF8;
  rb->setFrameAlgSize(net_width, net_height);

  Display display;
  display.start();

  VideoAnalysis app;
  app.init(size, net_width, net_height, display);
  
  app.run();

  vreader->stop();
  display.stop();

  delete rb;
  google::ShutdownGoogleLogging();

  return ret;
}

void sigHandle(int sig) {
  switch (sig) {
  case SIGINT:
    LOG(INFO) << "sigHandle:" << sig;
    break;
  default:
    LOG(INFO) << "sigHandle:" << sig;
    break;
  }
  ret = 1;
  b_run_main = false;
}
