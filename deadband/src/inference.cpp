#include "inference.h"

#define ONNX_YOLO_PATH "/weights/yolov7Tiny_640_640.onnx"
#define YOLO_INPUT_H 640
#define YOLO_INPUT_W 640
#define IMG_WIDTH 640
#define IMG_HEIGHT 480
#include <optional>
#include <algorithm>
#include <vector>

#include <opencv2/tracking.hpp>      // for KCF, CSRT, MIL
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
  if (src.depth() != CV_8U) {
    double alpha = 1.0, beta = 0.0;
    if (src.depth() == CV_16U)      alpha = 1.0 / 256.0;
    else if (src.depth() == CV_32F) alpha = 255.0;
    src.convertTo(tmp, CV_8U, alpha, beta);
  }

  cv::Mat dst;
  if (tmp.type() == CV_8UC2) {
    cv::cvtColor(tmp, dst, cv::COLOR_YUV2BGR_YUY2);
    return dst;
  }
  const int ch = tmp.channels();
  if (ch == 3) {
    dst = tmp;
  } else if (ch == 1) {
    cv::cvtColor(tmp, dst, cv::COLOR_GRAY2BGR);
  } else if (ch == 4) {
    cv::cvtColor(tmp, dst, cv::COLOR_BGRA2BGR);
  } else {
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
    else                      t = cv::TrackerKCF::create();  // fallback
  } catch (const cv::Exception& e) {
    std::cerr << "Tracker creation failed (" << type << "): " << e.what() << std::endl;
    t.release();
  } catch (...) {
    std::cerr << "Tracker creation failed (" << type << "): unknown exception" << std::endl;
    t.release();
  }
  return t;
}

OnnxInferenceNode::OnnxInferenceNode(const rclcpp::NodeOptions & options)
: Node("px2", options)
{
  // ----- Params -----
  this->declare_parameter<std::string>("device_path", "/dev/video2");
  this->declare_parameter<double>("fov", 70.0);
  this->declare_parameter<double>("kp", 1.0);
  this->declare_parameter<int>("lost_max_frames", 15);
  this->declare_parameter<int>("track_class", -1);
  this->declare_parameter<bool>("save_frames", false);
  this->declare_parameter<double>("max_step_deg", 8.0);
  this->declare_parameter<bool>("center_on_start", false);
  this->declare_parameter<std::string>("tracker_type", "KCF");
  this->declare_parameter<bool>("enforce_bgr8", true);

  device_path       = this->get_parameter("device_path").as_string();
  fov_              = this->get_parameter("fov").as_double();
  kp_               = this->get_parameter("kp").as_double();
  lost_max_frames_  = this->get_parameter("lost_max_frames").as_int();
  track_class_      = this->get_parameter("track_class").as_int();
  save_frames_      = this->get_parameter("save_frames").as_bool();
  max_step_deg_     = this->get_parameter("max_step_deg").as_double();
  center_on_start_  = this->get_parameter("center_on_start").as_bool();
  tracker_type_     = this->get_parameter("tracker_type").as_string();
  enforce_bgr8_     = this->get_parameter("enforce_bgr8").as_bool();

  // ----- Publisher for servo setpoints -----
  rclcpp::QoS qos(10);
  qos.reliable().durability_volatile(); // stream; don't latch servo commands
  pub_abs_ = this->create_publisher<custom_msgs::msg::AbsResult>("inference", qos);

  // ----- px3_ready (latched) -----
  rclcpp::QoS ready_qos(1);
  ready_qos.reliable().transient_local();
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
  cap_.open(device_path);
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

  // Run the pipeline once when allowed
  timer_ = this->create_wall_timer(50ms, std::bind(&OnnxInferenceNode::callbackInference, this));
}

std::pair<double, double> OnnxInferenceNode::computeCameraAngleFromBox(
    const cv::Rect& box, double HFOV_deg, double VFOV_deg) {
  const int box_cx = box.x + box.width  / 2;
  const int box_cy = box.y + box.height / 2;
  const int img_cx = IMG_WIDTH / 2;
  const int img_cy = IMG_HEIGHT / 2;
  const int dx = box_cx - img_cx;
  const int dy = box_cy - img_cy;

  double angle_x = (static_cast<double>(dx) / img_cx) * HFOV_deg;
  double angle_y = (static_cast<double>(dy) / img_cy) * VFOV_deg;
  if (!std::isfinite(angle_x)) angle_x = 0.0;
  if (!std::isfinite(angle_y)) angle_y = 0.0;
  return {angle_x, angle_y};
}

void OnnxInferenceNode::callbackInference()
{
  // start once, only after px3_ready
  if (started_ || !px3_ready_) return;
  started_ = true;

  RCLCPP_INFO(this->get_logger(), "[px2] Starting DETECT→TRACK pipeline.");

  // Do not publish here (race); we’ll send 90° once a subscriber is present
  bool initial_center_done = !center_on_start_;

  cv::Ptr<cv::Tracker> tracker;
  cv::Rect track_box;  // MUST be cv::Rect (int)
  int lost_frames = 0;

  enum class Mode { DETECT, TRACK };
  Mode mode = Mode::DETECT;

  int frame_id = 0;
  cv::Mat frame;

  while (rclcpp::ok() && cap_.read(frame)) {
    if (frame.empty()) continue;

    // Send initial 90° when we have a subscriber
    if (!initial_center_done && pub_abs_->get_subscription_count() > 0) {
      servo_deg_ = 90;
      publishState(90.0);
      initial_center_done = true;
      RCLCPP_INFO(this->get_logger(), "[px2] Initial center sent: 90°");
    }

    const cv::Size imsz = frame.size();
    auto [h_deg, v_deg] = computeFov(fov_, IMG_WIDTH, IMG_HEIGHT);
    const double hfov_deg = h_deg;
    const double vfov_deg = v_deg;

    if (mode == Mode::DETECT) {
      cv::Mat inputImage = yolo_->preprocess(frame, YOLO_INPUT_H, YOLO_INPUT_W);
      std::vector<Ort::Value> outputTensors = yolo_->RunInference(inputImage);
      std::vector<Result> results = yolo_->postprocess(imsz, outputTensors);

      if (save_frames_) {
        cv::Mat framec = frame.clone();
        cv::Mat vis = yolo_->drawBoundingBox(framec, results);
        cv::imwrite(pkg_path + "/data/detect_" + std::to_string(frame_id) + ".png", vis);
      }

      std::optional<Result> picked;
      int best_area = -1;
      for (const auto& r : results) {
        if (track_class_ >= 0 && r.obj_id != track_class_) continue;
        const int w = std::abs(r.x2 - r.x1);
        const int h = std::abs(r.y2 - r.y1);
        const int area = w * h;
        if (area > best_area) { best_area = area; picked = r; }
      }
      if (!picked) { ++frame_id; continue; }

      cv::Rect box(
        std::min(picked->x1, picked->x2),
        std::min(picked->y1, picked->y2),
        std::max(1, std::abs(picked->x2 - picked->x1)),
        std::max(1, std::abs(picked->y2 - picked->y1))
      );
      box &= cv::Rect(0, 0, IMG_WIDTH, IMG_HEIGHT);
      if (box.area() <= 1) { ++frame_id; continue; }

      auto [angle_x, angle_y] = computeCameraAngleFromBox(box, hfov_deg, vfov_deg);
      const int servo = servoSetpointFromError(angle_x);
      servo_deg_ = servo;
      RCLCPP_INFO(this->get_logger(), "servo angle is: %d° angle_x is: %.2f", servo, angle_x);
      publishState(static_cast<double>(servo));

      if (tracker_type_ == "none") { ++frame_id; continue; }

      tracker = make_tracker(tracker_type_);
      if (tracker.empty()) {
        RCLCPP_ERROR(this->get_logger(), "[px2] OpenCV tracker '%s' unavailable. Staying in DETECT.",
                     tracker_type_.c_str());
        ++frame_id; continue;
      }

      try {
        cv::Mat frame_bgr = to_bgr8(frame, enforce_bgr8_);
        tracker->init(frame_bgr, box);
        track_box   = box;     // keep cv::Rect (int)
        lost_frames = 0;
        mode        = Mode::TRACK;
        RCLCPP_INFO(this->get_logger(), "[px2] DETECT→TRACK (tracker=%s, servo=%d°)",
                    tracker_type_.c_str(), servo);
      } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "[px2] tracker->init threw: %s. Staying in DETECT.", e.what());
        tracker.release();
      }

    } else { // TRACK
      cv::Mat frame_bgr = to_bgr8(frame, enforce_bgr8_);
      const bool ok = tracker && tracker->update(frame_bgr, track_box);
      if (ok && track_box.width > 1 && track_box.height > 1) {
        track_box &= cv::Rect(0, 0, IMG_WIDTH, IMG_HEIGHT);
        auto [angle_x, angle_y] = computeCameraAngleFromBox(track_box, hfov_deg, vfov_deg);
        const int servo = servoSetpointFromError(angle_x);
        servo_deg_ = servo;
        RCLCPP_INFO(this->get_logger(), "servo angle is: %d° angle_x is: %.2f", servo, angle_x);
        publishState(static_cast<double>(servo));

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
