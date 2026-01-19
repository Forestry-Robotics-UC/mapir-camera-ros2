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
#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "camera_info_manager/camera_info_manager.hpp"
#include "mapir_camera_ros2/gstreamer_pipeline.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/srv/set_camera_info.hpp"

#include <opencv2/opencv.hpp>

namespace
{

cv::VideoCapture open_v4l2_capture(const std::string & video_device)
{
  if (!video_device.empty() && std::all_of(video_device.begin(), video_device.end(), ::isdigit)) {
    return cv::VideoCapture(std::stoi(video_device), cv::CAP_V4L2);
  }
  if (video_device.rfind("/dev/video", 0) == 0) {
    try {
      int index = std::stoi(video_device.substr(10));
      return cv::VideoCapture(index, cv::CAP_V4L2);
    } catch (const std::exception &) {
    }
  }
  return cv::VideoCapture(video_device, cv::CAP_V4L2);
}

}  // namespace

namespace mapir_camera_cpp
{

class MapirCameraCppNode : public rclcpp::Node {
public:
  MapirCameraCppNode()
  : Node("mapir_camera_cpp")
  {
    cv::setUseOptimized(true);
    declare_parameter("debug", false);
    declare_parameter("debug_period_s", 1.0);
    declare_parameter("video_device", "/dev/video0");
    declare_parameter("image_width", 1280);
    declare_parameter("image_height", 720);
    declare_parameter("framerate", 30.0);
    declare_parameter("frame_id", "mapir3_optical_frame");
    declare_parameter("camera_name", "mapir3_ocn");
    declare_parameter("camera_info_url", "");
    declare_parameter("pixel_format", "MJPG");
    declare_parameter("use_gstreamer", false);
    declare_parameter("gstreamer_pipeline", "");
    declare_parameter("reconnect_interval_s", 2.0);
    declare_parameter("use_capture_thread", true);
    declare_parameter("qos_depth", 5);
    declare_parameter("qos_best_effort", true);

    debug_ = get_parameter("debug").as_bool();
    debug_period_s_ = get_parameter("debug_period_s").as_double();

    video_device_ = get_parameter("video_device").as_string();
    req_width_ = get_parameter("image_width").as_int();
    req_height_ = get_parameter("image_height").as_int();
    req_fps_ = get_parameter("framerate").as_double();
    frame_id_ = get_parameter("frame_id").as_string();
    camera_name_ = get_parameter("camera_name").as_string();
    camera_info_url_ = get_parameter("camera_info_url").as_string();
    pixel_format_ = get_parameter("pixel_format").as_string();
    use_gstreamer_ = get_parameter("use_gstreamer").as_bool();
    gstreamer_pipeline_ = get_parameter("gstreamer_pipeline").as_string();
    reconnect_interval_s_ = get_parameter("reconnect_interval_s").as_double();
    use_capture_thread_ = get_parameter("use_capture_thread").as_bool();

    qos_depth_ = std::max(1, static_cast<int>(get_parameter("qos_depth").as_int()));
    qos_best_effort_ = get_parameter("qos_best_effort").as_bool();

    if (req_fps_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "Invalid framerate; defaulting to 30 Hz");
      req_fps_ = 30.0;
    }

    if (debug_) {
      log_table(
        "Camera Params",
        {
          {"video_device", video_device_},
          {"image_size", std::to_string(req_width_) + "x" + std::to_string(req_height_)},
          {"framerate", fmt_double(req_fps_, 2)},
          {"pixel_format", pixel_format_},
          {"use_gstreamer", use_gstreamer_ ? "true" : "false"},
          {"reconnect_interval_s", fmt_double(reconnect_interval_s_, 2)},
          {"use_capture_thread", use_capture_thread_ ? "true" : "false"},
          {"frame_id", frame_id_},
          {"camera_name", camera_name_},
          {"camera_info_url", camera_info_url_},
          {"debug_period_s", fmt_double(debug_period_s_, 2)},
        });
      log_table(
        "Publisher QoS",
        {
          {"reliability", qos_best_effort_ ? "BEST_EFFORT" : "RELIABLE"},
          {"depth", std::to_string(qos_depth_)},
          {"durability", "VOLATILE"},
        });
    }
    rclcpp::QoS qos_profile(qos_depth_);
    qos_profile.reliability(qos_best_effort_ ? RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT :
                                               RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", qos_profile);
    cinfo_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", qos_profile);

    cinfo_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
      this, camera_name_, camera_info_url_);
    if (!camera_info_url_.empty()) {
      RCLCPP_INFO(get_logger(), "camera calibration URL: %s", camera_info_url_.c_str());
      cinfo_manager_->loadCameraInfo(camera_info_url_);
    }
    if (debug_ && !(cinfo_manager_ && cinfo_manager_->isCalibrated())) {
      RCLCPP_WARN(get_logger(), "No valid calibration loaded; using default CameraInfo.");
    }

    try_open_camera(true);
    refresh_camera_info_cache();

    last_stats_t_ = std::chrono::steady_clock::now();
    last_fail_log_t_ = last_stats_t_;
    last_frame_t_ = last_stats_t_;

    if (use_capture_thread_) {
      running_.store(true);
      capture_thread_ = std::thread(&MapirCameraCppNode::capture_loop, this);
    } else {
      timer_period_ = std::chrono::duration<double>(1.0 / req_fps_);
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period_),
        std::bind(&MapirCameraCppNode::capture_and_publish, this));
    }

    set_camera_info_srv_ = create_service<sensor_msgs::srv::SetCameraInfo>(
      "set_camera_info",
      std::bind(
        &MapirCameraCppNode::handle_set_camera_info,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "MAPIR Survey3 Camera (C++) started: %dx%d @ %.1f Hz (%s)",
                width_, height_, req_fps_, video_device_.c_str());
  }

  ~MapirCameraCppNode() override
  {
    running_.store(false);
    if (cap_.isOpened()) {
      cap_.release();
    }
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
  }

private:
  bool open_camera()
  {
    std::string fmt = pixel_format_;
    for (auto & c : fmt) {
      c = static_cast<char>(::toupper(c));
    }

    if (use_gstreamer_) {
      GstreamerConfig cfg;
      cfg.device = video_device_;
      cfg.width = req_width_;
      cfg.height = req_height_;
      cfg.fps = static_cast<int>(std::round(req_fps_));
      cfg.pixel_format = fmt;
      cfg.custom_pipeline = gstreamer_pipeline_;
      std::string pipeline = BuildGstreamerPipeline(cfg);
      if (debug_) {
        RCLCPP_INFO(get_logger(), "GStreamer pipeline: %s", pipeline.c_str());
      }
      cap_.open(pipeline, cv::CAP_GSTREAMER);
      width_ = req_width_;
      height_ = req_height_;
    } else {
      cap_ = open_v4l2_capture(video_device_);
      if (!cap_.isOpened()) {
        return false;
      }
      if (fmt != "MJPG" && fmt != "H264") {
        RCLCPP_WARN(get_logger(), "Unsupported pixel_format '%s', forcing MJPG", fmt.c_str());
        fmt = "MJPG";
      }
      int fourcc = cv::VideoWriter::fourcc(fmt[0], fmt[1], fmt[2], fmt[3]);
      cap_.set(cv::CAP_PROP_FOURCC, fourcc);
      cap_.set(cv::CAP_PROP_FRAME_WIDTH, req_width_);
      cap_.set(cv::CAP_PROP_FRAME_HEIGHT, req_height_);
      cap_.set(cv::CAP_PROP_FPS, req_fps_);
      cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

      width_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
      height_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    }

    if (!cap_.isOpened()) {
      return false;
    }

    cv::Mat test_frame;
    if (cap_.read(test_frame) && !test_frame.empty()) {
      width_ = test_frame.cols;
      height_ = test_frame.rows;
      RCLCPP_INFO(get_logger(), "First frame OK: shape=%dx%d", width_, height_);
    } else {
      RCLCPP_WARN(get_logger(), "Camera opened but no frames could be read");
    }
    return true;
  }

  sensor_msgs::msg::CameraInfo default_camerainfo() const
  {
    sensor_msgs::msg::CameraInfo info;
    info.width = width_;
    info.height = height_;
    return info;
  }

  void refresh_camera_info_cache()
  {
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    if (cinfo_manager_ && cinfo_manager_->isCalibrated()) {
      camera_info_msg_ = cinfo_manager_->getCameraInfo();
    } else {
      camera_info_msg_ = default_camerainfo();
    }
    camera_info_msg_.width = width_;
    camera_info_msg_.height = height_;
  }

  bool try_open_camera(bool initial)
  {
    auto now = std::chrono::steady_clock::now();
    if (!initial && last_open_attempt_t_.time_since_epoch().count() > 0) {
      const double dt_s = std::chrono::duration<double>(now - last_open_attempt_t_).count();
      if (dt_s < reconnect_interval_s_) {
        return false;
      }
    }
    last_open_attempt_t_ = now;

    if (cap_.isOpened()) {
      cap_.release();
    }

    const bool opened = open_camera();
    if (!opened) {
      const double log_dt_s = std::chrono::duration<double>(
        now - last_open_log_t_).count();
      if (last_open_log_t_.time_since_epoch().count() == 0 ||
        log_dt_s >= reconnect_interval_s_)
      {
        RCLCPP_WARN(get_logger(), "Camera open failed; retrying...");
        last_open_log_t_ = now;
      }
      return false;
    }

    RCLCPP_INFO(get_logger(), "Camera connected: %s", video_device_.c_str());
    refresh_camera_info_cache();
    return true;
  }

  void capture_loop()
  {
    while (rclcpp::ok() && running_.load()) {
      if (!cap_.isOpened()) {
        if (!try_open_camera(false)) {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        continue;
      }
      if (!cap_.read(frame_) || frame_.empty()) {
        fail_reads_ += 1;
        if (debug_) {
          auto now = std::chrono::steady_clock::now();
          const double dt_s = std::chrono::duration<double>(now - last_fail_log_t_).count();
          if (dt_s >= debug_period_s_) {
            RCLCPP_WARN(get_logger(), "Camera read failed (fail_reads=%d)", fail_reads_);
            last_fail_log_t_ = now;
          }
        }
        continue;
      }
      publish_frame(frame_);
    }
  }

  void capture_and_publish()
  {
    if (!cap_.isOpened()) {
      try_open_camera(false);
      return;
    }

    if (!cap_.read(frame_) || frame_.empty()) {
      fail_reads_ += 1;
      if (debug_) {
        auto now = std::chrono::steady_clock::now();
        const double dt_s = std::chrono::duration<double>(now - last_fail_log_t_).count();
        if (dt_s >= debug_period_s_) {
          RCLCPP_WARN(get_logger(), "Camera read failed (fail_reads=%d)", fail_reads_);
          last_fail_log_t_ = now;
        }
      }
      return;
    }
    publish_frame(frame_);
  }

  void publish_frame(const cv::Mat & frame)
  {
    const cv::Mat *frame_ptr = &frame;
    if (frame.type() != CV_8UC3) {
      if (frame.channels() == 1) {
        cv::cvtColor(frame, bgr_frame_, cv::COLOR_GRAY2BGR);
        frame_ptr = &bgr_frame_;
      } else if (frame.channels() == 4) {
        cv::cvtColor(frame, bgr_frame_, cv::COLOR_BGRA2BGR);
        frame_ptr = &bgr_frame_;
      }
    }

    if (frame_ptr->cols != width_ || frame_ptr->rows != height_) {
      width_ = frame_ptr->cols;
      height_ = frame_ptr->rows;
      refresh_camera_info_cache();
    }

    auto stamp = get_clock()->now();
    auto now = std::chrono::steady_clock::now();
    if (last_frame_t_.time_since_epoch().count() > 0) {
      const double dt_s = std::chrono::duration<double>(now - last_frame_t_).count();
      if (dt_s > max_inter_frame_s_) {
        max_inter_frame_s_ = dt_s;
      }
    }
    last_frame_t_ = now;

    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = frame_id_;

    const size_t step = static_cast<size_t>(frame_ptr->cols) * frame_ptr->elemSize();
    const size_t size = step * static_cast<size_t>(frame_ptr->rows);
    if (image_msg_.data.size() != size) {
      image_msg_.data.resize(size);
    }
    image_msg_.header = header;
    image_msg_.height = static_cast<uint32_t>(frame_ptr->rows);
    image_msg_.width = static_cast<uint32_t>(frame_ptr->cols);
    image_msg_.encoding = "bgr8";
    image_msg_.is_bigendian = false;
    image_msg_.step = static_cast<uint32_t>(step);
    if (frame_ptr->isContinuous()) {
      std::memcpy(image_msg_.data.data(), frame_ptr->data, size);
    } else {
      const int rows = frame_ptr->rows;
      const size_t row_bytes = static_cast<size_t>(frame_ptr->cols) * frame_ptr->elemSize();
      for (int row = 0; row < rows; ++row) {
        const uint8_t *src = frame_ptr->ptr<uint8_t>(row);
        uint8_t *dst = image_msg_.data.data() + row * row_bytes;
        std::memcpy(dst, src, row_bytes);
      }
    }

    image_pub_->publish(image_msg_);
    {
      std::lock_guard<std::mutex> lock(camera_info_mutex_);
      camera_info_msg_.header = header;
      camera_info_msg_.width = static_cast<uint32_t>(width_);
      camera_info_msg_.height = static_cast<uint32_t>(height_);
      cinfo_pub_->publish(camera_info_msg_);
    }
    pub_frames_ += 1;

    if (debug_) {
      const double stats_dt_s = std::chrono::duration<double>(now - last_stats_t_).count();
      if (stats_dt_s >= debug_period_s_) {
        const int frames_since = pub_frames_ - last_stats_pub_frames_;
        const double est_hz = frames_since / std::max(1e-6, stats_dt_s);
        log_table(
          "Camera Stats",
          {
            {"published_frames", std::to_string(pub_frames_)},
            {"fail_reads", std::to_string(fail_reads_)},
            {"est_pub_hz", fmt_double(est_hz, 2)},
            {"max_inter_frame_dt_s", fmt_double(max_inter_frame_s_, 4)},
          });
        last_stats_pub_frames_ = pub_frames_;
        last_stats_t_ = now;
        max_inter_frame_s_ = 0.0;
      }
    }
  }

  std::string fmt_double(double value, int precision) const
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
  }

  void log_table(
    const std::string & title,
    const std::vector<std::pair<std::string, std::string>> & rows)
  {
    if (rows.empty()) {
      return;
    }
    size_t key_width = 0;
    size_t val_width = 0;
    for (const auto & row : rows) {
      key_width = std::max(key_width, row.first.size());
      val_width = std::max(val_width, row.second.size());
    }
    std::ostringstream header;
    header << std::left << std::setw(static_cast<int>(key_width)) << "Param"
           << " | " << std::left << std::setw(static_cast<int>(val_width)) << "Value";
    const std::string header_str = header.str();
    const std::string sep(header_str.size(), '-');
    std::ostringstream table;
    table << title << '\n' << header_str << '\n' << sep;
    for (const auto & row : rows) {
      std::ostringstream line;
      line << std::left << std::setw(static_cast<int>(key_width)) << row.first
           << " | " << std::left << std::setw(static_cast<int>(val_width)) << row.second;
      table << '\n' << line.str();
    }
    RCLCPP_INFO(get_logger(), "%s", table.str().c_str());
  }

  void handle_set_camera_info(
    const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> request,
    std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> response)
  {
    if (!cinfo_manager_) {
      response->success = false;
      response->status_message = "CameraInfoManager not initialized.";
      return;
    }
    response->success = cinfo_manager_->setCameraInfo(request->camera_info);
    if (!response->success) {
      response->status_message = "Failed to store camera calibration.";
      return;
    }
    refresh_camera_info_cache();
  }

  bool debug_ = false;
  double debug_period_s_ = 1.0;

  std::string video_device_;
  int req_width_ = 1280;
  int req_height_ = 720;
  double req_fps_ = 30.0;
  std::string frame_id_;
  std::string camera_name_;
  std::string camera_info_url_;
  std::string pixel_format_;
  bool use_gstreamer_ = false;
  std::string gstreamer_pipeline_;
  double reconnect_interval_s_ = 2.0;
  bool use_capture_thread_ = true;

  int qos_depth_ = 5;
  bool qos_best_effort_ = true;

  int width_ = 0;
  int height_ = 0;
  int fail_reads_ = 0;
  int pub_frames_ = 0;
  int last_stats_pub_frames_ = 0;
  double max_inter_frame_s_ = 0.0;
  std::chrono::steady_clock::time_point last_stats_t_;
  std::chrono::steady_clock::time_point last_fail_log_t_;
  std::chrono::steady_clock::time_point last_frame_t_;

  cv::VideoCapture cap_;
  cv::Mat frame_;
  cv::Mat bgr_frame_;

  sensor_msgs::msg::Image image_msg_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  std::mutex camera_info_mutex_;

  std::atomic<bool> running_{false};
  std::thread capture_thread_;
  std::chrono::steady_clock::time_point last_open_attempt_t_;
  std::chrono::steady_clock::time_point last_open_log_t_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cinfo_pub_;
  rclcpp::Service<sensor_msgs::srv::SetCameraInfo>::SharedPtr set_camera_info_srv_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;

  std::chrono::duration<double> timer_period_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mapir_camera_cpp

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mapir_camera_cpp::MapirCameraCppNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
