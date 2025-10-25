#ifndef ONNX_INFERENCE_HPP_
#define ONNX_INFERENCE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <atomic>
#include <cmath>  // std::lround, std::abs
#include <std_msgs/msg/bool.hpp>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <algorithm>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/abs_result.hpp"

// Root of your package on disk. Adjust if different.
#define PKG_PATH "/ros2_ws/work/src/px2/"

#include "yolo_inference.h"  // declares Result and YoloDetect
#include "tracker_tool.h"
#include "composition_tool.h"

// Exact pixel->yaw mapping: theta = atan( 2*tan(HFOV/2) * (dx/W) )
static inline double pixelErrorToYawDeg(double dx_px, double width_px, double hfov_deg)
{
    const double PI   = 3.14159265358979323846;
    const double hfov = hfov_deg * PI / 180.0;
    const double yaw_rad = std::atan( (2.0 * std::tan(0.5 * hfov)) * (dx_px / width_px) );
    return yaw_rad * 180.0 / PI;
}

// Undo letterbox padding from NET (netW x netH) back to RAW (imgW x imgH)
static inline cv::Rect2f unletterboxRect(const cv::Rect2f& r_net,
                                         int imgW, int imgH,
                                         int netW, int netH)
{
    const float s    = std::min(netW / float(imgW), netH / float(imgH));
    const int   newW = int(std::round(imgW * s));
    const int   newH = int(std::round(imgH * s));
    const float padX = (netW - newW) * 0.5f;
    const float padY = (netH - newH) * 0.5f;

    cv::Rect2f r;
    r.x      = (r_net.x - padX) / s;
    r.y      = (r_net.y - padY) / s;
    r.width  =  r_net.width  / s;
    r.height =  r_net.height / s;
    r &= cv::Rect2f(0,0,(float)imgW,(float)imgH);
    return r;
}


namespace onnx_inference
{
class OnnxInferenceNode : public rclcpp::Node
{
public:
  explicit OnnxInferenceNode();

private:
  // ===== ROS I/O =====
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<custom_msgs::msg::AbsResult>::SharedPtr pub_abs_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr px3_ready_sub_;

  // --- ACK from px3 ---
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr px3_ack_sub_;
  rclcpp::CallbackGroup::SharedPtr ack_group_;
  std::mutex ack_mtx_;
  std::condition_variable ack_cv_;
  bool ack_ready_ = false;

  // ===== Config / resources =====
  std::string pkg_path = PKG_PATH;
  std::string device_path;
  cv::VideoCapture cap_;
  std::unique_ptr<YoloDetect> yolo_;

  // ===== Flow / state =====
  std::atomic<bool> px3_ready_{false};
  bool started_ = false;               // ensure the long loop runs once
  double  servo_deg_ = 90;                // last commanded servo angle
  float hfov_deg = 53.1;

  // ===== Tracking / control params =====
  int    lost_max_frames_ = 15;
  bool   save_frames_ = false;
  std::string tracker_type_ = "KCF";   // "KCF" | "CSRT" | "none"
  bool   enforce_bgr8_ = true;         // convert frames to CV_8UC3 for tracker
  ThirdsTarget thirds_target_ = ::ThirdsTarget::CENTER;
  bool draw_thirds_overlay_{true};
  // ===== Main pipeline (DETECT ↔ TRACK) =====
  void callbackInference();
  bool wait_for_ack_ms(int timeout_ms);

  // ===== Utilities =====
  void publishState(double deg);
};

}  // namespace onnx_inference
#endif  // ONNX_INFERENCE_HPP_