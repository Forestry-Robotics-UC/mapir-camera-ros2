#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "camera_info_manager/camera_info_manager.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "mapir_camera_cpp/gstreamer_pipeline.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/opencv.hpp>

namespace {

cv::VideoCapture open_v4l2_capture(const std::string &video_device) {
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

namespace mapir_camera_cpp {

class MapirCameraCppNode : public rclcpp::Node {
 public:
  MapirCameraCppNode()
  : Node("mapir_camera_cpp") {
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

    qos_depth_ = std::max(1, static_cast<int>(get_parameter("qos_depth").as_int()));
    qos_best_effort_ = get_parameter("qos_best_effort").as_bool();

    if (req_fps_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "Invalid framerate; defaulting to 30 Hz");
      req_fps_ = 30.0;
    }

    if (debug_) {
      RCLCPP_INFO(get_logger(), "Params: device=%s size=%dx%d fps=%.2f fmt=%s frame_id=%s camera_name=%s",
                  video_device_.c_str(), req_width_, req_height_, req_fps_, pixel_format_.c_str(),
                  frame_id_.c_str(), camera_name_.c_str());
      RCLCPP_INFO(get_logger(), "QoS: reliability=%s depth=%d",
                  qos_best_effort_ ? "BEST_EFFORT" : "RELIABLE", qos_depth_);
    }
    rclcpp::QoS qos_profile(qos_depth_);
    qos_profile.reliability(qos_best_effort_ ? RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
                                             : RMW_QOS_POLICY_RELIABILITY_RELIABLE);
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

    open_camera();

    last_stats_t_ = std::chrono::steady_clock::now();
    last_fail_log_t_ = last_stats_t_;
    last_frame_t_ = last_stats_t_;

    timer_period_ = std::chrono::duration<double>(1.0 / req_fps_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period_),
      std::bind(&MapirCameraCppNode::capture_and_publish, this));

    RCLCPP_INFO(get_logger(), "MAPIR Survey3 Camera (C++) started: %dx%d @ %.1f Hz (%s)",
                width_, height_, req_fps_, video_device_.c_str());
  }

  ~MapirCameraCppNode() override {
    if (cap_.isOpened()) {
      cap_.release();
    }
  }

 private:
  void open_camera() {
    std::string fmt = pixel_format_;
    for (auto &c : fmt) {
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
        throw std::runtime_error("Failed to open video device");
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

      width_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
      height_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    }

    if (!cap_.isOpened()) {
      throw std::runtime_error("Camera open failed");
    }

    cv::Mat test_frame;
    if (cap_.read(test_frame) && !test_frame.empty()) {
      width_ = test_frame.cols;
      height_ = test_frame.rows;
      RCLCPP_INFO(get_logger(), "First frame OK: shape=%dx%d", width_, height_);
    } else {
      RCLCPP_WARN(get_logger(), "Camera opened but no frames could be read");
    }
  }

  sensor_msgs::msg::CameraInfo default_camerainfo() const {
    sensor_msgs::msg::CameraInfo info;
    info.width = width_;
    info.height = height_;
    return info;
  }

  void capture_and_publish() {
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
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

    auto img_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

    sensor_msgs::msg::CameraInfo cinfo;
    if (cinfo_manager_ && cinfo_manager_->isCalibrated()) {
      cinfo = cinfo_manager_->getCameraInfo();
    } else {
      cinfo = default_camerainfo();
    }
    cinfo.header = header;

    image_pub_->publish(*img_msg);
    cinfo_pub_->publish(cinfo);
    pub_frames_ += 1;

    if (debug_) {
      const double stats_dt_s = std::chrono::duration<double>(now - last_stats_t_).count();
      if (stats_dt_s >= debug_period_s_) {
        const int frames_since = pub_frames_ - last_stats_pub_frames_;
        const double est_hz = frames_since / std::max(1e-6, stats_dt_s);
        RCLCPP_INFO(get_logger(),
                    "Stats: published_frames=%d fail_reads=%d est_pub_hz=%.2f max_inter_frame_dt=%.4fs",
                    pub_frames_, fail_reads_, est_hz, max_inter_frame_s_);
        last_stats_pub_frames_ = pub_frames_;
        last_stats_t_ = now;
        max_inter_frame_s_ = 0.0;
      }
    }
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

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cinfo_pub_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;

  std::chrono::duration<double> timer_period_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mapir_camera_cpp

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mapir_camera_cpp::MapirCameraCppNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
