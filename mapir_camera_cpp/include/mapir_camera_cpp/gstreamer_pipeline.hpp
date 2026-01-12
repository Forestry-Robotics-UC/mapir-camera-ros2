#ifndef MAPIR_CAMERA_CPP_GSTREAMER_PIPELINE_HPP
#define MAPIR_CAMERA_CPP_GSTREAMER_PIPELINE_HPP

#include <sstream>
#include <string>

namespace mapir_camera_cpp {

struct GstreamerConfig {
  std::string device;
  int width = 1280;
  int height = 720;
  int fps = 30;
  std::string pixel_format;  // "MJPG" or "H264"
  std::string custom_pipeline;
};

inline std::string BuildGstreamerPipeline(const GstreamerConfig &cfg) {
  if (!cfg.custom_pipeline.empty()) {
    return cfg.custom_pipeline;
  }

  std::ostringstream pipeline;

  if (cfg.pixel_format == "H264") {
    pipeline << "v4l2src device=" << cfg.device << " ! "
             << "video/x-h264,width=" << cfg.width << ",height=" << cfg.height
             << ",framerate=" << cfg.fps << "/1 ! "
             << "h264parse ! "
             << "avdec_h264 ! "
             << "videoconvert ! "
             << "video/x-raw,format=BGR ! "
             << "appsink drop=true max-buffers=1 sync=false";
    return pipeline.str();
  }

  pipeline << "v4l2src device=" << cfg.device << " ! "
           << "image/jpeg,width=" << cfg.width << ",height=" << cfg.height
           << ",framerate=" << cfg.fps << "/1 ! "
           << "jpegdec ! "
           << "videoconvert ! "
           << "video/x-raw,format=BGR ! "
           << "appsink drop=true max-buffers=1 sync=false";
  return pipeline.str();
}

}  // namespace mapir_camera_cpp

#endif  // MAPIR_CAMERA_CPP_GSTREAMER_PIPELINE_HPP
