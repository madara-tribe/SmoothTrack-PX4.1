#ifndef ONNX_INFERENCE_HPP_
#define ONNX_INFERENCE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <atomic>
#include <cmath>  // std::lround, std::abs
#include <std_msgs/msg/bool.hpp>
#include <mutex>
#include <algorithm>
#include <vector>

#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/abs_result.hpp"

// Root of your package on disk. Adjust if different.
#define PKG_PATH "/ros2_ws/work/src/px2/"

#include "yolo_inference.h"
#include "sort_tracker.h" 
#include "composition_tool.h"
#include "utils.h"

// Exact pixel->yaw mapping: theta = atan( 2*tan(HFOV/2) * (dx/W) )
static inline double pixelErrorToYawDeg(double dx_px, double width_px, double hfov_deg)
{
    const double PI   = 3.14159265358979323846;
    const double hfov = hfov_deg * PI / 180.0;
    const double yaw_rad = std::atan( (2.0 * std::tan(0.5 * hfov)) * (dx_px / width_px) );
    return yaw_rad * 180.0 / PI;
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
  bool   enforce_bgr8_ = true;         // convert frames to CV_8UC3 for tracker
  ThirdsTarget thirds_target_ = ::ThirdsTarget::CENTER;
  bool preproc_enable_{false};
  // ===== Main pipeline (DETECT ↔ TRACK) =====
  void callbackInference();
  bool wait_for_ack_ms(int timeout_ms);
  
  // ===== Utilities =====
  void publishState(double deg);
  void saveThirdsOverlayIfNeeded(const cv::Mat& frame,
                                 const cv::Rect2d& roi_img, int frame_id,
                                 double target_x, const std::string& name_prefix);
};

}  // namespace onnx_inference
#endif  // ONNX_INFERENCE_HPP_