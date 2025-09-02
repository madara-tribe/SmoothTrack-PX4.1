#include "inference.h"

#define ONNX_YOLO_PATH "/weights/yolov7Tiny_640_640.onnx"
#define YOLO_INPUT_H 640
#define YOLO_INPUT_W 640

#include <optional>
#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/tracking.hpp>   // CSRT/KCF live here (opencv_contrib tracking)
#include <opencv2/imgproc.hpp>

using namespace std::chrono_literals;

namespace onnx_inference
{
  // Convert any common frame to CV_8UC3 (BGR). If enforce_bgr8=false, returns src.
  // Convert any common frame to CV_8UC3 (BGR). If enforce_bgr8=false, returns src.
static inline cv::Mat to_bgr8(const cv::Mat& src, bool enforce_bgr8)
{
  if (!enforce_bgr8) return src;
  if (src.type() == CV_8UC3) return src;

  cv::Mat tmp = src;
  // 1) Convert depth to 8U if needed
  if (src.depth() != CV_8U) {
    double alpha = 1.0, beta = 0.0;
    if (src.depth() == CV_16U)      alpha = 1.0 / 256.0;  // 16U -> 8U
    else if (src.depth() == CV_32F) alpha = 255.0;        // 32F -> 8U
    src.convertTo(tmp, CV_8U, alpha, beta);
  }

  // 2) Convert channels to 3 (BGR)
  cv::Mat dst;
  const int ch = tmp.channels();
  if (ch == 3) {
    dst = tmp;  // already BGR8
  } else if (ch == 1) {
    cv::cvtColor(tmp, dst, cv::COLOR_GRAY2BGR);
  } else if (ch == 4) {
    cv::cvtColor(tmp, dst, cv::COLOR_BGRA2BGR);
  } else {
    // Fallback: keep the first 3 channels
    std::vector<cv::Mat> channels;
    cv::split(tmp, channels);
    while (channels.size() < 3) channels.push_back(channels.back());
    cv::merge(std::vector<cv::Mat>{channels[0], channels[1], channels[2]}, dst);
  }
  return dst;
}
  // Make tracker by type string
  static cv::Ptr<cv::Tracker> make_tracker(const std::string& type)
  {
    cv::Ptr<cv::Tracker> t;
    try {
      if (type == "KCF")      t = cv::TrackerKCF::create();
      else if (type == "CSRT") t = cv::TrackerCSRT::create();
      else /* none / unknown */ t.release();
    } catch (...) {
      t.release();
    }
    return t; // may be empty if tracking module isn't present
  }

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

    // Control-mode params (Option C friendly)
    this->declare_parameter<std::string>("control_mode", "abs"); // "abs" or "inc"
    this->declare_parameter<double>("kpx_deg_per_px", 1.0);      // only for "inc"
    this->declare_parameter<int>("deadband_px", 3);
    this->declare_parameter<double>("max_step_deg", 8.0);
    this->declare_parameter<bool>("center_on_start", true);

    // Tracker params
    this->declare_parameter<std::string>("tracker_type", "KCF"); // "KCF" | "CSRT" | "none"
    this->declare_parameter<bool>("enforce_bgr8", true);

    device_path       = this->get_parameter("device_path").as_string();
    hfov_deg_         = this->get_parameter("hfov_deg").as_double();
    vfov_deg_         = this->get_parameter("vfov_deg").as_double();
    kp_               = this->get_parameter("kp").as_double();
    invert_servo_     = this->get_parameter("invert_servo").as_bool();
    lost_max_frames_  = this->get_parameter("lost_max_frames").as_int();
    track_class_      = this->get_parameter("track_class").as_int();
    save_frames_      = this->get_parameter("save_frames").as_bool();
    control_mode_     = this->get_parameter("control_mode").as_string();
    kpx_deg_per_px_   = this->get_parameter("kpx_deg_per_px").as_double();
    deadband_px_      = this->get_parameter("deadband_px").as_int();
    max_step_deg_     = this->get_parameter("max_step_deg").as_double();
    center_on_start_  = this->get_parameter("center_on_start").as_bool();
    tracker_type_     = this->get_parameter("tracker_type").as_string();
    enforce_bgr8_     = this->get_parameter("enforce_bgr8").as_bool();

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

    // ----- Open camera -----
    RCLCPP_INFO(this->get_logger(), "Opening camera at: %s", device_path.c_str());
    cap_.open(device_path); // optionally: cap_.open(device_path, cv::CAP_V4L2);
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
    // ensure we start only once, and only after px3_ready flips us on
    if (started_ || !inference_triggered_) {
      return;
    }
    started_ = true;
    inference_triggered_ = false;

    RCLCPP_INFO(this->get_logger(), "[px2] Starting DETECT→TRACK pipeline. Waiting for px3_ready=%s",
                px3_ready_ ? "true" : "false");

    // --- Wait until px3_ready observed ---
    while (rclcpp::ok() && !px3_ready_) {
      rclcpp::sleep_for(50ms);
    }
    RCLCPP_INFO(this->get_logger(), "[px2] px3_ready confirmed. Running…");

    // Optional: ensure servo starts at 90° once
    if (center_on_start_) {
      custom_msgs::msg::AbsResult init;
      init.x_angle = 90.0;
      publishState(init);
      servo_deg_ = 90;
    }

    // Prepare YOLO
    YoloDetect yolo_detector(pkg_path + ONNX_YOLO_PATH);
    yolo_detector.setTrackingMode(SINGLE);
    TrackingMode current_mode = yolo_detector.getTrackingMode();

    // Prepare tracker
    cv::Ptr<cv::Tracker> tracker;
    cv::Rect track_box;   // int Rect to match common update() signatures
    int lost_frames = 0;

    enum class Mode { DETECT, TRACK };
    Mode mode = Mode::DETECT;

    int frame_id = 0;
    cv::Mat frame;

    while (rclcpp::ok() && cap_.read(frame)) {
      if (frame.empty()) break;
      const cv::Size imsz = frame.size();

      // Convert for tracker if needed
      cv::Mat frame_bgr = to_bgr8(frame, enforce_bgr8_);

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
          // bbox sanity
          cv::Rect box(
            std::min(picked->x1, picked->x2),
            std::min(picked->y1, picked->y2),
            std::max(1, std::abs(picked->x2 - picked->x1)),
            std::max(1, std::abs(picked->y2 - picked->y1))
          );
          box &= cv::Rect(0, 0, imsz.width, imsz.height);
          if (box.area() <= 1) {
            RCLCPP_WARN(this->get_logger(), "[px2] DETECT degenerate bbox; stay in DETECT");
          } else {
            // compute error → servo
            int img_cx = imsz.width / 2;
            int bx_cx  = (picked->x1 + picked->x2) / 2;
            int dx     = bx_cx - img_cx;

            int servo = 90;
            if (control_mode_ == "inc") {
              servo = servoUpdateIncremental(dx);  // deadband + rate limit
            } else { // "abs"
              auto [ax, ay] = computeCameraAngleFromBox(*picked, imsz, hfov_deg_, vfov_deg_);
              int desired = servoSetpointFromError(ax);
              servo = rateLimitAndClamp(desired);
            }

            custom_msgs::msg::AbsResult msg;
            msg.x_angle = static_cast<double>(servo);  // ABSOLUTE setpoint
            publishState(msg);

            // If tracking disabled, remain in DETECT
            if (tracker_type_ == "none") {
              RCLCPP_WARN(this->get_logger(),
                "[px2] tracker_type=none → staying in DETECT (no OpenCV tracker).");
            } else {
              // Make tracker safely
              tracker = make_tracker(tracker_type_);
              if (tracker.empty()) {
                RCLCPP_ERROR(this->get_logger(),
                  "[px2] OpenCV tracker '%s' unavailable. Staying in DETECT.",
                  tracker_type_.c_str());
              } else {
                // init() may segfault in bad builds; we minimize risk by forcing BGR8
                try {
                  tracker->init(frame_bgr, box);  // some builds return void; fine
                  track_box   = box;
                  lost_frames = 0;
                  mode        = Mode::TRACK;
                  RCLCPP_INFO(this->get_logger(), "[px2] DETECT→TRACK (tracker=%s, servo=%d°)",
                              tracker_type_.c_str(), servo);
                } catch (const cv::Exception& e) {
                  RCLCPP_ERROR(this->get_logger(),
                    "[px2] tracker->init threw: %s. Staying in DETECT.", e.what());
                  tracker.release();
                }
              }
            }
          }
        }

        if (save_frames_) {
          std::string out = pkg_path + std::string("/data/frames/detect_") + std::to_string(frame_id) + ".png";
          cv::imwrite(out, vis);
        }

      } else { // TRACK mode
        bool ok = false;
        try {
          ok = tracker && tracker->update(frame_bgr, track_box);
        } catch (const cv::Exception& e) {
          RCLCPP_ERROR(this->get_logger(), "[px2] tracker->update threw: %s", e.what());
          ok = false;
        }

        if (!ok ||
            track_box.area() <= 1 ||
            track_box.x < 0 || track_box.y < 0 ||
            track_box.x + track_box.width  > imsz.width ||
            track_box.y + track_box.height > imsz.height) {
          lost_frames++;
        } else {
          lost_frames = 0;

          int img_cx = imsz.width / 2;
          int bx_cx  = static_cast<int>(track_box.x + track_box.width * 0.5);
          int dx     = bx_cx - img_cx;

          int servo = 90;
          if (control_mode_ == "inc") {
            servo = servoUpdateIncremental(dx);  // deadband + rate limit
          } else { // "abs"
            double ax  = (static_cast<double>(dx) / imsz.width) * hfov_deg_;
            int desired = servoSetpointFromError(ax);
            servo = rateLimitAndClamp(desired);
          }

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
    double s = std::clamp(static_cast<double>(message.x_angle), 0.0, 180.0);
    RCLCPP_INFO(this->get_logger(), "Publishing servo setpoint: %.2f°", s);
    custom_msgs::msg::AbsResult out = message;
    out.x_angle = s;
    publisher_->publish(out);
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

