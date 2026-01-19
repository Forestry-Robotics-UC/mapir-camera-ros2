// Copyright 2025 Duda Andrada
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
#ifndef MAPIR_CAMERA_ROS2__GSTREAMER_PIPELINE_HPP_
#define MAPIR_CAMERA_ROS2__GSTREAMER_PIPELINE_HPP_

#include <sstream>
#include <string>

namespace mapir_camera_cpp
{

struct GstreamerConfig
{
  std::string device;
  int width = 1280;
  int height = 720;
  int fps = 30;
  std::string pixel_format;  // "MJPG" or "H264"
  std::string custom_pipeline;
};

inline std::string BuildGstreamerPipeline(const GstreamerConfig & cfg)
{
  if (!cfg.custom_pipeline.empty()) {
    return cfg.custom_pipeline;
  }

  std::ostringstream pipeline;

  if (cfg.pixel_format == "H264") {
    pipeline << "v4l2src device=" << cfg.device << " io-mode=2 ! "
             << "video/x-h264,width=" << cfg.width << ",height=" << cfg.height
             << ",framerate=" << cfg.fps
             << "/1,stream-format=byte-stream,alignment=au ! "
             << "queue max-size-buffers=1 leaky=downstream ! "
             << "h264parse config-interval=-1 ! "
             << "avdec_h264 ! "
             << "videoconvert ! "
             << "video/x-raw,format=BGR ! "
             << "appsink drop=true max-buffers=1 sync=false";
    return pipeline.str();
  }

  pipeline << "v4l2src device=" << cfg.device << " io-mode=2 ! "
           << "image/jpeg,width=" << cfg.width << ",height=" << cfg.height
           << ",framerate=" << cfg.fps << "/1 ! "
           << "queue max-size-buffers=1 leaky=downstream ! "
           << "jpegdec ! "
           << "videoconvert ! "
           << "video/x-raw,format=BGR ! "
           << "appsink drop=true max-buffers=1 sync=false";
  return pipeline.str();
}

}  // namespace mapir_camera_cpp

#endif  // MAPIR_CAMERA_ROS2__GSTREAMER_PIPELINE_HPP_
