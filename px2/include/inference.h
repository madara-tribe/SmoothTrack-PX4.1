#ifndef ONNX_INFERENCE_HPP_
#define ONNX_INFERENCE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <atomic>
#include <cmath>  // std::lround, std::abs

#include <std_msgs/msg/bool.hpp>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/abs_result.hpp"

// Root of your package on disk. Adjust if different.
#define PKG_PATH "/ros2_ws/work/src/px2/"

#include "yolo_inference.h"  // declares Result and YoloDetect

// Compute horizontal & vertical FOV from diagonal FOV and image aspect ratio
std::pair<double, double> computeFov(double diag_fov_deg, int width, int height) {
    const double aspect = static_cast<double>(width) / height;
    const double diag_fov_rad = diag_fov_deg * M_PI / 180.0;

    // compute half-angle tangents
    const double tan_diag_half = std::tan(diag_fov_rad / 2.0);
    const double tan_hfov_half = tan_diag_half / std::sqrt(1.0 + 1.0 / (aspect * aspect));
    const double tan_vfov_half = tan_hfov_half / aspect;

    const double hfov_deg = 2.0 * std::atan(tan_hfov_half) * 180.0 / M_PI;
    const double vfov_deg = 2.0 * std::atan(tan_vfov_half) * 180.0 / M_PI;
    return {hfov_deg, vfov_deg};
}

namespace onnx_inference
{
class OnnxInferenceNode : public rclcpp::Node
{
public:
  explicit OnnxInferenceNode(const rclcpp::NodeOptions & options);

private:
  // ===== ROS I/O =====
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<custom_msgs::msg::AbsResult>::SharedPtr pub_abs_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr px3_ready_sub_;

  // ===== Config / resources =====
  std::string pkg_path = PKG_PATH;
  std::string device_path;
  cv::VideoCapture cap_;
  std::unique_ptr<YoloDetect> yolo_;

  // ===== Flow / state =====
  std::atomic<bool> px3_ready_{false};
  bool started_ = false;               // ensure the long loop runs once
  int  servo_deg_ = 90;                // last commanded servo angle

  // ===== Tracking / control params =====
  double fov_ = 70.0;
  double kp_       = 1.0;              // abs-mode gain
  int    lost_max_frames_ = 15;
  int    track_class_ = -1;            // -1 => any class
  bool   save_frames_ = false;
  double max_step_deg_ = 8.0;          // max deg/update (rate limit)
  bool   center_on_start_ = false;     // px3 already centers to 90°
  std::string tracker_type_ = "KCF";   // "KCF" | "CSRT" | "none"
  bool   enforce_bgr8_ = true;         // convert frames to CV_8UC3 for tracker

  // ===== Main pipeline (DETECT ↔ TRACK) =====
  void callbackInference();

  // ===== Utilities =====
  std::pair<double, double> computeCameraAngleFromBox(
      const cv::Rect& box,
      double HFOV_deg = 90.0, double VFOV_deg = 59.0);

  void publishState(double deg);

  inline int servoSetpointFromError(double angle_x_deg) const {
    // 90° when centered; depend on the kp_, servo flips 
    double s = 90.0 + (kp_ * angle_x_deg);
    return std::clamp(s, 0.0, 180.0);
  }
};

}  // namespace onnx_inference
#endif  // ONNX_INFERENCE_HPP_