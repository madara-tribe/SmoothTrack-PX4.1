#ifndef ONNX_INFERENCE_HPP_
#define ONNX_INFERENCE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
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
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<custom_msgs::msg::AbsResult>::SharedPtr publisher_;
  std::string pkg_path = PKG_PATH;
  std::string device_path;
  cv::VideoCapture cap_;
  bool inference_triggered_ = false;
  void callbackInference();
  void stringCallback(const std_msgs::msg::String::SharedPtr msg);
  std::pair<double, double>computeCameraAngleFromBox(const Result& result, const cv::Size& imageSize, double HFOV_deg = 56.8, double VFOV_deg = 44.0);
  void publishState(const custom_msgs::msg::AbsResult & message);
};

}  // namespace onnx_inference
#endif  // ONNX_INFERENCE_HPP_
