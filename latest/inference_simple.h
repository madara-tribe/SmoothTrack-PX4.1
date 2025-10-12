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

namespace onnx_inference
{
class OnnxInferenceNode : public rclcpp::Node
{
public:
  explicit OnnxInferenceNode(const rclcpp::NodeOptions & options);

private:
  // ===== ROS I/O =====
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<custom_msgs::msg::AbsResult>::SharedPtr publisher_;
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
  double hfov_deg_ = 62.0;
  double vfov_deg_ = 48.0;
  double kp_       = 1.0;              // abs-mode gain
  bool   invert_servo_ = false;
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
      const Result& result,
      const cv::Size& imageSize,
      double HFOV_deg = 90.0, double VFOV_deg = 59.0);

  void publishState(double deg);

  inline int servoSetpointFromError(double angle_x_deg) const {
    // 90° when centered; invert_servo_ flips sign if needed (usually false because px3 flips)
    double s = 90.0 + (invert_servo_ ? -kp_ : kp_) * angle_x_deg;
    if (s < 0.0) s = 0.0;
    if (s > 180.0) s = 180.0;
    return static_cast<int>(std::lround(s));
  }

  // Rate‑limit absolute setpoint toward desired, update servo_deg_
  inline int rateLimitAndClamp(int desired_deg) {
    int delta = desired_deg - servo_deg_;
    int step  = static_cast<int>(std::lround(max_step_deg_));
    if (delta >  step) desired_deg = servo_deg_ + step;
    if (delta < -step) desired_deg = servo_deg_ - step;
    if (desired_deg < 0)   desired_deg = 0;
    if (desired_deg > 180) desired_deg = 180;
    servo_deg_ = desired_deg;
    return servo_deg_;
  }
};

}  // namespace onnx_inference
#endif  // ONNX_INFERENCE_HPP_