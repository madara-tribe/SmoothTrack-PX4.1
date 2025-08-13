#ifndef ONNX_INFERENCE_HPP_
#define ONNX_INFERENCE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <atomic>
#include <cmath>  // for std::lround

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/abs_result.hpp"

#define PKG_PATH "/ros2_ws/work/src/px2/"

#include "yolo_inference.h"

namespace onnx_inference
{
class OnnxInferenceNode : public rclcpp::Node
{
public:
  explicit OnnxInferenceNode(const rclcpp::NodeOptions & options);

private:
  // ROS I/O
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<custom_msgs::msg::AbsResult>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr px3_ready_sub_;

  // Config / resources
  std::string pkg_path = PKG_PATH;
  std::string device_path;
  cv::VideoCapture cap_;

  // Flow / state
  std::atomic<bool> px3_ready_{false};
  bool started_ = false;               // ensure the long loop runs once
  bool inference_triggered_ = false;   // set true when px3_ready observed

  // Tracking / control params (declared via parameters)
  double hfov_deg_ = 62.0;
  double vfov_deg_ = 48.0;
  double kp_       = 1.0;     // deg of servo per deg of image error (abs mode)
  bool invert_servo_ = false;
  int   lost_max_frames_ = 15;
  int   track_class_ = -1;    // -1 => any class
  bool  save_frames_ = false;

  // Control mode (optional: "abs" or "inc")
  std::string control_mode_ = "abs";   // "abs" (default) or "inc"
  double kpx_deg_per_px_    = 1.0;     // used only in "inc" mode
  bool   center_on_start_   = true;    // publish 90° once after handshake
  int    servo_deg_         = 90;      // last commanded servo angle (for "inc")

  // Main pipeline (DETECT ↔ TRACK)
  void callbackInference();

  // Utilities
  std::pair<double, double> computeCameraAngleFromBox(
      const Result& result,
      const cv::Size& imageSize,
      double HFOV_deg = 90.0, double VFOV_deg = 59.0);

  void publishState(const custom_msgs::msg::AbsResult & message);

  // Control helpers
  inline int servoSetpointFromError(double angle_x_deg) const {
    double s = 90.0 + (invert_servo_ ? -kp_ : kp_) * angle_x_deg;
    if (s < 0.0) {
      s = 0.0;
    } else if (s > 180.0) {
      s = 180.0;
    }
    return static_cast<int>(std::lround(s));
  }

  inline int servoUpdateIncremental(int dx_px) {
    double delta = (invert_servo_ ? -1.0 : 1.0) * kpx_deg_per_px_ * static_cast<double>(dx_px);
    servo_deg_ += static_cast<int>(std::lround(delta));
    if (servo_deg_ < 0) {
      servo_deg_ = 0;
    } else if (servo_deg_ > 180) {
      servo_deg_ = 180;
    }
    return servo_deg_;
  }
};

}  // namespace onnx_inference
#endif  // ONNX_INFERENCE_HPP_

