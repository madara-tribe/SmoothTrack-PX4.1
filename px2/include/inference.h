#ifndef ONNX_INFERENCE_HPP_
#define ONNX_INFERENCE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <atomic>
#include <cmath>  // std::lround, std::abs
#include <algorithm>   // std::clamp
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
  double  servo_deg_ = 90;                // last commanded servo angle

  // ===== Tracking / control params =====
  double min_angle_       = 0.0;
  double max_angle_ = 180.0;          // max deg/update (rate limit)      
  int    lost_max_frames_ = 15;
  bool   save_frames_ = false;
  std::string tracker_type_ = "KCF";   // "KCF" | "CSRT" | "none"
  bool   enforce_bgr8_ = true;         // convert frames to CV_8UC3 for tracker

  // ===== Main pipeline (DETECT ↔ TRACK) =====
  void callbackInference();

  // ===== Utilities =====
  double computeServoAngleFromBox(const cv::Rect& box);
  void publishState(double deg);
};

}  // namespace onnx_inference
#endif  // ONNX_INFERENCE_HPP_