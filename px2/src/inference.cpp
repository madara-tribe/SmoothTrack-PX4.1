#include "inference.h"

#define ONNX_YOLO_PATH "/weights/yolov7Tiny_640_640.onnx"
#define YOLO_INPUT_H 640
#define YOLO_INPUT_W 640

using namespace std::chrono_literals;

namespace onnx_inference
{
  OnnxInferenceNode::OnnxInferenceNode(const rclcpp::NodeOptions & options)
  : Node("px2", options), inference_triggered_(true)  // Trigger inference immediately
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("inference", rclcpp::QoS{10}.transient_local());
    timer_ = this->create_wall_timer(
        500ms, std::bind(&OnnxInferenceNode::callbackInference, this));
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/string_trigger", rclcpp::QoS{10},
      std::bind(&OnnxInferenceNode::stringCallback, this, std::placeholders::_1));
  }

  void OnnxInferenceNode::stringCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "yes") {
      inference_triggered_ = true;
    }
  }

  void OnnxInferenceNode::callbackInference()
  {
    if (!inference_triggered_) {
      return;
    }
    inference_triggered_ = false;
    RCLCPP_INFO(rclcpp::get_logger("ImageSubscriber"), "Running YOLO inference");

    cv::VideoCapture cap(pkg_path + "/data/output.mp4");
    if (!cap.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open video file: output.mp4");
        return;
    }

    YoloDetect yolo_detector(pkg_path + ONNX_YOLO_PATH);
    yolo_detector.setTrackingMode(SINGLE);
    //yolo_detector.setTrackingMode(GROUP);
    int frame_id = 0;
    cv::Mat frame;
      while (cap.read(frame)) {
          if (frame.empty()) break;
          auto start = std::chrono::high_resolution_clock::now();
          
          cv::Mat inputImage = yolo_detector.preprocess(frame, YOLO_INPUT_H, YOLO_INPUT_W);
          std::vector<Ort::Value> outputTensors = yolo_detector.RunInference(inputImage);
          std::vector<Result> resultVector = yolo_detector.postprocess(frame.size(), outputTensors);
          cv::Mat yolo_result = yolo_detector.drawBoundingBox(frame, resultVector);
          
          auto end = std::chrono::high_resolution_clock::now();
          std::chrono::duration<double> diff = end - start;
          
          std::cout << "[Frame " << frame_id << "] YOLO inference took " << diff.count() << " seconds" << std::endl;
          
          // Optional: save or show result
          std::string output_path = pkg_path + "/data/frames/frame_" + std::to_string(frame_id) + ".png";
          cv::imwrite(output_path, yolo_result);
          
          frame_id++;
      }
    std_msgs::msg::String message;
    message.data = "inference_done";
    publishState(message);
  }

  void OnnxInferenceNode::publishState(const std_msgs::msg::String & message)
  {
     RCLCPP_INFO(this->get_logger(), "Publishing inference completion message");
     publisher_->publish(message);
  }
}  // namespace onnx_inference

int main(int argc, char * argv[])
{
  rclcpp::NodeOptions options;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<onnx_inference::OnnxInferenceNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
