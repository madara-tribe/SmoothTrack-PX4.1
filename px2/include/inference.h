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
  // https://item.rakuten.co.jp/soushin-shop/webcam01-cp/?ultra_crid=webcam01-cp&variantId=webcam01-cp&scid=af_sp_etc&sc2id=af_113_0_10001868&icm_acid=255-776-8501&icm_cid=15280804517&iasid=wem_icbs_&gclid=CjwKCAjwkbzEBhAVEiwA4V-yqkyRkEOcjA42f33CtHbXbLaLR905EJi3t0qAmNl6OlGJAYjDc_StAxoC6XcQAvD_BwE&ifd=57&gbraid=0AAAAADoVjpiuZ5phOqRZoy2D6VcJYMU0U&icm_agid=135083843412
  std::pair<double, double>computeCameraAngleFromBox(const Result& result, const cv::Size& imageSize, double HFOV_deg = 90.0, double VFOV_deg = 59.0);
  void publishState(const custom_msgs::msg::AbsResult & message);
};

}  // namespace onnx_inference
#endif  // ONNX_INFERENCE_HPP_
