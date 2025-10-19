#include "inference.h"

#define ONNX_YOLO_PATH "/weights/yolov7Tiny_640_640.onnx"
#define YOLO_INPUT_H 640
#define YOLO_INPUT_W 640
#define IMG_WIDTH 960
#define IMG_HEIGHT 540
#define CENTER_TOLERANCE 266
#include <optional>
#include <algorithm>
#include <vector>

#include <opencv2/tracking.hpp>      // for KCF, CSRT, MIL, MedianFlow
#include <opencv2/core/ocl.hpp>
#include <opencv2/imgproc.hpp>

using namespace std::chrono_literals;

namespace onnx_inference
{
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
  if (tmp.type() == CV_8UC2) {                     // YUYV (YUY2)
    cv::cvtColor(tmp, dst, cv::COLOR_YUV2BGR_YUY2);
    return dst;
  }
  const int ch = tmp.channels();
  if (ch == 3) {
    dst = tmp;                                      // already BGR8
  } else if (ch == 1) {
    cv::cvtColor(tmp, dst, cv::COLOR_GRAY2BGR);
  } else if (ch == 4) {
    cv::cvtColor(tmp, dst, cv::COLOR_BGRA2BGR);
  } else {
    // Fallback: keep first 3 channels
    std::vector<cv::Mat> channels;
    cv::split(tmp, channels);
    while (channels.size() < 3) channels.push_back(channels.back());
    cv::merge(std::vector<cv::Mat>{channels[0], channels[1], channels[2]}, dst);
  }
  return dst;
}

static cv::Ptr<cv::Tracker> make_tracker(const std::string& type)
{
  cv::Ptr<cv::Tracker> t;
  try {
    std::string up = type;
    std::transform(up.begin(), up.end(), up.begin(), ::toupper);

    if (up == "KCF")          t = cv::TrackerKCF::create();
    else if (up == "CSRT")    t = cv::TrackerCSRT::create();
    else if (up == "MIL")     t = cv::TrackerMIL::create();
    else {
      // optional: fallback to KCF
      t = cv::TrackerKCF::create();
    }
  } catch (const cv::Exception& e) {
    std::cerr << "Tracker creation failed (" << type << "): " << e.what() << std::endl;
    t.release();
  } catch (...) {
    std::cerr << "Tracker creation failed (" << type << "): unknown exception" << std::endl;
    t.release();
  }
  return t;  // may be empty if not available in your build
}

OnnxInferenceNode::OnnxInferenceNode(const rclcpp::NodeOptions & options)
: Node("px2", options)
{
  // ----- Params -----
  this->declare_parameter<std::string>("device_path", "/dev/video2");
  this->declare_parameter<double>("min_angle", 0.0);
  this->declare_parameter<double>("max_angle", 180.0);
  this->declare_parameter<int>("lost_max_frames", 15);
  this->declare_parameter<bool>("save_frames", false);
  this->declare_parameter<bool>("center_on_start", false); // px3 centers
  this->declare_parameter<std::string>("tracker_type", "KCF"); // "KCF" | "CSRT" | "none"
  this->declare_parameter<bool>("enforce_bgr8", true);

  device_path       = this->get_parameter("device_path").as_string();
  min_angle_               = this->get_parameter("min_angle").as_double();
  max_angle_     = this->get_parameter("max_angle").as_double();
  lost_max_frames_  = this->get_parameter("lost_max_frames").as_int();
  save_frames_      = this->get_parameter("save_frames").as_bool();
  center_on_start_  = this->get_parameter("center_on_start").as_bool();
  tracker_type_     = this->get_parameter("tracker_type").as_string();
  enforce_bgr8_     = this->get_parameter("enforce_bgr8").as_bool();

  // ----- Publisher for servo setpoints -----
  rclcpp::QoS qos(10);
  qos.reliable().durability_volatile(); // stream; don't latch servo commands
  pub_abs_ = this->create_publisher<custom_msgs::msg::AbsResult>("inference", qos);

  // ----- px3_ready (latched) -----
  rclcpp::QoS ready_qos(1);
  ready_qos.reliable().transient_local();  // receive latched True
  px3_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "px3_ready", ready_qos,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      if (msg->data) {
        px3_ready_ = true;
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
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, IMG_WIDTH);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT);

  // ---- Load YOLO once ----
  yolo_ = std::make_unique<YoloDetect>(pkg_path + std::string(ONNX_YOLO_PATH));
  // Optional: if your YoloDetect supports modes
  // yolo_->setTrackingMode(SINGLE);

  // Run the pipeline once when allowed
  timer_ = this->create_wall_timer(50ms, std::bind(&OnnxInferenceNode::callbackInference, this));
}

double OnnxInferenceNode::computeServoAngleFromBox(
    const cv::Rect& box) {
  const int box_cx = box.x + box.width  / 2;
  const int box_cy = box.y + box.height / 2;
  const int img_cx = IMG_WIDTH / 2;
  const int img_cy = IMG_HEIGHT / 2;
  std::cout << "img_cx : " << img_cx << "img_cy: "<< img_cy << "box._cx: " << box_cx << ": box_cy"<< box_cy  << std::endl;
  int offset_width = box_cx - img_cx;
  //const int offset_height = box_cy - img_cy;
  std::cout << "offset_width :" << offset_width << std::endl;
  // Check if the object is centered
  //if (offset_width < CENTER_TOLERANCE){
  if (offset_width > 0){
    // Object is to the right of the center, rotate servo right
    servo_deg_ = std::min(servo_deg_ + 5, max_angle_);
    } else if (offset_width < 0) {
    // Object is to the left of the center, rotate servo left
    servo_deg_ = std::max(servo_deg_ - 5, min_angle_);
  }
  //}
  return servo_deg_;
}

void OnnxInferenceNode::callbackInference()
{
  // ensure we start only once, and only after px3_ready flips us on
  if (started_ || !px3_ready_) return;
  started_ = true;

  RCLCPP_INFO(this->get_logger(), "[px2] Starting DETECT→TRACK pipeline.");

  // Optional: ensure servo starts at 90° once
  if (center_on_start_) {
    servo_deg_ = 90.0;
    publishState(90.0);
  }

  // Prepare tracker runtime
  cv::Ptr<cv::Tracker> tracker;
  cv::Rect track_box;
  int lost_frames = 0;

  enum class Mode { DETECT, TRACK };
  Mode mode = Mode::DETECT;

  int frame_id = 0;
  cv::Mat frame;
  
  while (rclcpp::ok() && cap_.read(frame)) {
    if (frame.empty()) continue;
    const cv::Size imsz = frame.size();

    if (mode == Mode::DETECT) {
      // YOLO until target appears
      cv::Mat inputImage = yolo_->preprocess(frame, YOLO_INPUT_H, YOLO_INPUT_W);
      std::vector<Ort::Value> outputTensors = yolo_->RunInference(inputImage);
      std::vector<Result> results = yolo_->postprocess(imsz, outputTensors);

      if (save_frames_) {
        cv::Mat vis = yolo_->drawBoundingBox(frame, results);
        cv::imwrite(pkg_path + "/data/detect_" + std::to_string(frame_id) + ".png", vis);
      }

      // pick one candidate (largest area; filter by class if requested)
      std::optional<Result> picked;
      int best_area = -1;
      for (const auto& r : results) {
        int w = std::abs(r.x2 - r.x1);
        int h = std::abs(r.y2 - r.y1);
        int area = w * h;
        if (area > best_area) { best_area = area; picked = r; }
      }
      if (!picked) { ++frame_id; continue; }

      // bbox sanity
      cv::Rect box(
        std::min(picked->x1, picked->x2),
        std::min(picked->y1, picked->y2),
        std::max(1, std::abs(picked->x2 - picked->x1)),
        std::max(1, std::abs(picked->y2 - picked->y1))
      );
      box &= cv::Rect(0, 0, IMG_WIDTH, IMG_HEIGHT);
      if (box.area() <= 1) { ++frame_id; continue; }

      // compute error → servo (horizontal only)
      servo_deg_ = computeServoAngleFromBox(box);
      RCLCPP_INFO(this->get_logger(), "servo angle is: %.2f°", servo_deg_);
      publishState(servo_deg_);

      // If tracking disabled, remain in DETECT
      if (tracker_type_ == "none") { ++frame_id; continue; }

      // Start tracker
      tracker = make_tracker(tracker_type_);
      if (tracker.empty()) {
        RCLCPP_ERROR(this->get_logger(), "[px2] OpenCV tracker '%s' unavailable. Staying in DETECT.",
                     tracker_type_.c_str());
        ++frame_id; continue;
      }
    
      // init() on BGR8 for stability
      try {
        cv::Mat frame_bgr = to_bgr8(frame, enforce_bgr8_);
        tracker->init(frame_bgr, box);
        track_box   = cv::Rect2d(box);
        lost_frames = 0;
        mode        = Mode::TRACK;
        RCLCPP_INFO(this->get_logger(), "[px2] DETECT→TRACK (tracker=%s, servo_deg_=%d°)",
                    tracker_type_.c_str(), servo_deg_);
      } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "[px2] tracker->init threw: %s. Staying in DETECT.", e.what());
        tracker.release();
      }
    } else { // TRACK mode
      // TRACK uses BGR8
      cv::Rect2d cur = track_box;
      cv::Mat frame_bgr = to_bgr8(frame, enforce_bgr8_);
      bool ok = tracker && tracker->update(frame_bgr, track_box);
      if (ok && cur.width > 1.0 && cur.height > 1.0) {
        servo_deg_ = computeServoAngleFromBox(track_box);
        RCLCPP_INFO(this->get_logger(), "servo_deg_ is: %.2f°", servo_deg_);
        publishState(servo_deg_);
        if (save_frames_) {
          cv::Mat out = frame.clone();
          cv::rectangle(out, track_box, {0,255,0}, 2);
          cv::imwrite(pkg_path + "/data/track_" + std::to_string(frame_id) + ".png", out);
        }
        lost_frames = 0;
      } else {
        if (++lost_frames >= lost_max_frames_) {
          RCLCPP_WARN(this->get_logger(), "[px2] TRACK lost → DETECT after %d frames.", lost_frames);
          tracker.release();
          lost_frames = 0;
          mode = Mode::DETECT;
        }
      }
    } 
    ++frame_id;
  }
  RCLCPP_INFO(this->get_logger(), "[px2] pipeline finished.");
}

void OnnxInferenceNode::publishState(double deg)
{
  custom_msgs::msg::AbsResult m;
  m.x_angle = deg;
  RCLCPP_INFO(this->get_logger(), "Publishing servo setpoint: %.2f°", m.x_angle);
  pub_abs_->publish(m);
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
