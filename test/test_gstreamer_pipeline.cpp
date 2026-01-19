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
#include <gtest/gtest.h>

#include "mapir_camera_ros2/gstreamer_pipeline.hpp"

using mapir_camera_cpp::BuildGstreamerPipeline;
using mapir_camera_cpp::GstreamerConfig;

TEST(GstreamerPipeline, UsesCustomPipelineWhenProvided) {
  GstreamerConfig cfg;
  cfg.custom_pipeline = "custom pipeline";
  auto pipeline = BuildGstreamerPipeline(cfg);
  EXPECT_EQ(pipeline, "custom pipeline");
}

TEST(GstreamerPipeline, BuildsMJPGPipeline) {
  GstreamerConfig cfg;
  cfg.device = "/dev/video0";
  cfg.width = 1280;
  cfg.height = 720;
  cfg.fps = 30;
  cfg.pixel_format = "MJPG";
  auto pipeline = BuildGstreamerPipeline(cfg);
  EXPECT_NE(pipeline.find("image/jpeg"), std::string::npos);
  EXPECT_NE(pipeline.find("jpegdec"), std::string::npos);
}

TEST(GstreamerPipeline, BuildsH264Pipeline) {
  GstreamerConfig cfg;
  cfg.device = "/dev/video0";
  cfg.width = 1280;
  cfg.height = 720;
  cfg.fps = 30;
  cfg.pixel_format = "H264";
  auto pipeline = BuildGstreamerPipeline(cfg);
  EXPECT_NE(pipeline.find("video/x-h264"), std::string::npos);
  EXPECT_NE(pipeline.find("avdec_h264"), std::string::npos);
}
