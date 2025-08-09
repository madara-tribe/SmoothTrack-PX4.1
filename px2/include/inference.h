##ifndef ONNX_INFERENCE_HPP_
#define ONNX_INFERENCE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
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
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_; // keep (unused trigger)

  // Config / resources
  std::string pkg_path = PKG_PATH;
  std::string device_path;
  cv::VideoCapture cap_;

  // Flow / state
  bool inference_triggered_ = false;     // start only after px3_ready
  std::atomic<bool> px3_ready_{false};
  bool started_ = false;

  // Tracking params (declared via parameters)
  double hfov_deg_ = 62.0;
  double vfov_deg_ = 48.0;
  double kp_       = 1.0;
  bool invert_servo_ = false;
  int   lost_max_frames_ = 15;
  int   track_class_ = -1;      // -1 => any class
  bool  save_frames_ = false;

  // Main pipeline
  void callbackInference(); // long-running DETECT↔TRACK loop
  void stringCallback(const std_msgs::msg::String::SharedPtr msg);

  // Utilities
  std::pair<double, double> computeCameraAngleFromBox(
      const Result& result,
      const cv::Size& imageSize,
      double HFOV_deg = 90.0, double VFOV_deg = 59.0);
  void publishState(const custom_msgs::msg::AbsResult & message);

  inline int servoSetpointFromError(double angle_x_deg) const {
    double s = 90.0 + (invert_servo_ ? -kp_ : kp_) * angle_x_deg;
    if (s < 0.0) s = 0.0;
    if (s > 180.0) s = 180.0;
    return static_cast<int>(std::lround(s));
  }
};

}  // namespace onnx_inference
#endif  // ONNX_INFERENCE_HPP_
