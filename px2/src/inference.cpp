#include "inference.h"

#define ONNX_YOLO_PATH "/weights/yolov7Tiny_640_640.onnx"
#define YOLO_INPUT_H 640
#define YOLO_INPUT_W 640

#include <opencv2/tracking.hpp>   // needs opencv_contrib for CSRT; falls back to KCF

using namespace std::chrono_literals;

namespace onnx_inference
{
  OnnxInferenceNode::OnnxInferenceNode(const rclcpp::NodeOptions & options)
  : Node("px2", options)
  {
    // ----- Params -----
    this->declare_parameter<std::string>("device_path", "/dev/video2");
    this->declare_parameter<double>("hfov_deg", 62.0);
    this->declare_parameter<double>("vfov_deg", 48.0);
    this->declare_parameter<double>("kp", 1.0);
    this->declare_parameter<bool>("invert_servo", false);
    this->declare_parameter<int>("lost_max_frames", 15);
    this->declare_parameter<int>("track_class", -1);     // -1 => any
    this->declare_parameter<bool>("save_frames", false);

    device_path      = this->get_parameter("device_path").as_string();
    hfov_deg_        = this->get_parameter("hfov_deg").as_double();
    vfov_deg_        = this->get_parameter("vfov_deg").as_double();
    kp_              = this->get_parameter("kp").as_double();
    invert_servo_    = this->get_parameter("invert_servo").as_bool();
    lost_max_frames_ = this->get_parameter("lost_max_frames").as_int();
    track_class_     = this->get_parameter("track_class").as_int();
    save_frames_     = this->get_parameter("save_frames").as_bool();

    // ----- Publisher for servo setpoints -----
    rclcpp::QoS qos(10);
    qos.reliable().durability_volatile(); // stream; don't latch servo commands
    publisher_ = this->create_publisher<custom_msgs::msg::AbsResult>("inference", qos);

    // ----- px3_ready (latched) -----
    rclcpp::QoS ready_qos(1);
    ready_qos.reliable().transient_local();  // receive latched True
    px3_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "px3_ready", ready_qos,
      [this](const std_msgs::msg::Bool::SharedPtr msg){
        if (msg->data) {
          px3_ready_ = true;
          inference_triggered_ = true; // allow the main loop to start
          RCLCPP_INFO(this->get_logger(), "px2: received px3_ready=True");
        }
      });

    // Optional string trigger (kept from your file)
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/string_trigger", rclcpp::QoS{10},
      std::bind(&OnnxInferenceNode::stringCallback, this, std::placeholders::_1));

    // ----- Open camera -----
    RCLCPP_INFO(this->get_logger(), "Opening camera at: %s", device_path.c_str());
    cap_.open(device_path);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera device: %s", device_path.c_str());
      rclcpp::shutdown();
      return;
    }
    cap_.set(cv::CAP_PROP_FPS, 30);
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    // Run the pipeline once when allowed
    timer_ = this->create_wall_timer(50ms, std::bind(&OnnxInferenceNode::callbackInference, this));
  }

  void OnnxInferenceNode::stringCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "yes") {
      inference_triggered_ = true;
    }
  }

  std::pair<double, double> OnnxInferenceNode::computeCameraAngleFromBox(
      const Result& result, const cv::Size& imageSize, double HFOV_deg, double VFOV_deg) {
    int box_cx = (result.x1 + result.x2) / 2;
    int box_cy = (result.y1 + result.y2) / 2;
    int img_cx = imageSize.width / 2;
    int img_cy = imageSize.height / 2;

    int dx = box_cx - img_cx;
    int dy = box_cy - img_cy;

    double angle_x = (static_cast<double>(dx) / imageSize.width)  * HFOV_deg;
    double angle_y = (static_cast<double>(dy) / imageSize.height) * VFOV_deg;
    return {angle_x, angle_y};
  }

  void OnnxInferenceNode::callbackInference()
  {
    if (started_ || !inference_triggered_) {
      return;
    }
    started_ = true;          // ensure we only enter once
    inference_triggered_ = false;
    RCLCPP_INFO(this->get_logger(), "[px2] Starting DETECT→TRACK pipeline. Waiting for px3_ready=%s",
                px3_ready_ ? "true" : "false");

    // --- Wait (just in case) until px3_ready observed ---
    while (rclcpp::ok() && !px3_ready_) {
      rclcpp::sleep_for(50ms);
    }
    RCLCPP_INFO(this->get_logger(), "[px2] px3_ready confirmed. Running…");

    // Prepare YOLO
    YoloDetect yolo_detector(pkg_path + ONNX_YOLO_PATH);
    yolo_detector.setTrackingMode(SINGLE);
    TrackingMode current_mode = yolo_detector.getTrackingMode();

    // Prepare tracker
    cv::Ptr<cv::Tracker> tracker;
    cv::Rect2d track_box;
    int lost_frames = 0;

    enum class Mode { DETECT, TRACK };
    Mode mode = Mode::DETECT;

    int frame_id = 0;
    cv::Mat frame;

    while (rclcpp::ok() && cap_.read(frame)) {
      if (frame.empty()) break;
      const cv::Size imsz = frame.size();

      if (mode == Mode::DETECT) {
        // YOLO until target appears
        cv::Mat inputImage = yolo_detector.preprocess(frame, YOLO_INPUT_H, YOLO_INPUT_W);
        std::vector<Ort::Value> outputTensors = yolo_detector.RunInference(inputImage);
        std::vector<Result> results = yolo_detector.postprocess(imsz, outputTensors);
        cv::Mat vis = yolo_detector.drawBoundingBox(frame, results);

        // pick one candidate (by class if requested)
        std::optional<Result> picked;
        for (const auto& r : results) {
          if (track_class_ >= 0 && r.obj_id != track_class_) continue;
          bool ok = (current_mode == SINGLE && r.obj_id != -1) ||
                    (current_mode == GROUP  && r.obj_id == -1);
          if (ok) { picked = r; break; }
        }

        if (picked) {
          // compute error and publish absolute servo setpoint
          auto [ax, ay] = computeCameraAngleFromBox(*picked, imsz, hfov_deg_, vfov_deg_);
          int servo = servoSetpointFromError(ax);

          custom_msgs::msg::AbsResult msg;
          msg.x_angle = static_cast<double>(servo);  // ABSOLUTE setpoint
          publishState(msg);

          // init tracker
          cv::Rect2d box(
            std::min(picked->x1, picked->x2),
            std::min(picked->y1, picked->y2),
            std::abs(picked->x2 - picked->x1),
            std::abs(picked->y2 - picked->y1)
          );
          try { tracker = cv::TrackerCSRT::create(); }
          catch (...) { tracker = cv::TrackerKCF::create(); }

          if (!tracker->init(frame, box)) {
            RCLCPP_WARN(this->get_logger(), "[px2] tracker init failed; stay in DETECT");
          } else {
            track_box = box;
            lost_frames = 0;
            mode = Mode::TRACK;
            RCLCPP_INFO(this->get_logger(), "[px2] DETECT→TRACK (servo=%d°, err_x=%.2f°)", servo, ax);
          }
        }

        if (save_frames_) {
          std::string out = pkg_path + std::string("/data/frames/detect_") + std::to_string(frame_id) + ".png";
          cv::imwrite(out, vis);
        }

      } else { // TRACK mode
        bool ok = tracker->update(frame, track_box);
        if (!ok || track_box.area() <= 1.0 ||
            track_box.x < 0 || track_box.y < 0 ||
            track_box.x + track_box.width  > imsz.width ||
            track_box.y + track_box.height > imsz.height) {
          lost_frames++;
        } else {
          lost_frames = 0;

          // error based on bbox center vs image center
          int img_cx = imsz.width / 2;
          int bx_cx  = static_cast<int>(track_box.x + track_box.width * 0.5);
          int dx     = bx_cx - img_cx;
          double ax  = (static_cast<double>(dx) / imsz.width) * hfov_deg_;

          int servo = servoSetpointFromError(ax);
          custom_msgs::msg::AbsResult msg;
          msg.x_angle = static_cast<double>(servo);
          publishState(msg);

          if (save_frames_) {
            cv::rectangle(frame, track_box, {0,255,0}, 2);
            std::string out = pkg_path + std::string("/data/frames/track_") + std::to_string(frame_id) + ".png";
            cv::imwrite(out, frame);
          }
        }

        if (lost_frames >= lost_max_frames_) {
          RCLCPP_WARN(this->get_logger(), "[px2] TRACK→DETECT (lost %d frames)", lost_frames);
          tracker.release();
          mode = Mode::DETECT;
        }
      }

      frame_id++;
    }

    RCLCPP_INFO(this->get_logger(), "[px2] pipeline finished.");
  }

  void OnnxInferenceNode::publishState(const custom_msgs::msg::AbsResult & message)
  {
    RCLCPP_INFO(this->get_logger(), "Publishing servo setpoint: %.2f°", message.x_angle);
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
