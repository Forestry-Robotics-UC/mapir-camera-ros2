#include "mapir_camera_cpp/gstreamer_pipeline.hpp"

#include <gtest/gtest.h>

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
