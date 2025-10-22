#include "inference.h"

#define ONNX_YOLO_PATH "/weights/yolov7Tiny_640_640.onnx"
#define YOLO_INPUT_H 640
#define YOLO_INPUT_W 640
#define IMG_WIDTH 960
#define IMG_HEIGHT 540
#define ARK_TIME 500

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

OnnxInferenceNode::OnnxInferenceNode()
: Node("px2")
{
  // ----- Params -----
  this->declare_parameter<std::string>("device_path", "/dev/video2");
  this->declare_parameter<int>("lost_max_frames", 15);
  this->declare_parameter<bool>("save_frames", false);
  this->declare_parameter<std::string>("tracker_type", "KCF"); // "KCF" | "CSRT" | "none"
  this->declare_parameter<bool>("enforce_bgr8", true);

  device_path       = this->get_parameter("device_path").as_string();
  lost_max_frames_  = this->get_parameter("lost_max_frames").as_int();
  save_frames_      = this->get_parameter("save_frames").as_bool();
  tracker_type_     = this->get_parameter("tracker_type").as_string();
  enforce_bgr8_     = this->get_parameter("enforce_bgr8").as_bool();

  // ----- Publisher for servo setpoints -----
  rclcpp::QoS qos(10);
  qos.reliable().durability_volatile(); // stream; don't latch servo commands
  pub_abs_ = this->create_publisher<custom_msgs::msg::AbsResult>("inference", qos);

  // Create a reentrant group so ACK callback can run while we wait
  ack_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions opts;
  opts.callback_group = ack_group_;
  
  px3_ack_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "px3_ack", rclcpp::QoS(10),
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        if (!m->data) return;
        {
          std::lock_guard<std::mutex> lk(ack_mtx_);
          ack_ready_ = true;
        }
        ack_cv_.notify_one();
      },
      opts);

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

  // Run the pipeline once when allowed
  timer_ = this->create_wall_timer(50ms, std::bind(&OnnxInferenceNode::callbackInference, this));
}

// Block until ACK or timeout
bool OnnxInferenceNode::wait_for_ack_ms(int timeout_ms)
{
  std::unique_lock<std::mutex> lk(ack_mtx_);
  ack_ready_ = false;
  return ack_cv_.wait_for(
      lk, std::chrono::milliseconds(timeout_ms),
      [this]{ return ack_ready_; });
}

void OnnxInferenceNode::callbackInference()
{
  // ensure we start only once, and only after px3_ready flips us on
  if (started_ || !px3_ready_) return;
  started_ = true;

  RCLCPP_INFO(this->get_logger(), "[px2] Starting DETECT→TRACK pipeline.");

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
      cv::Rect2f box(
        std::min(picked->x1, picked->x2),
        std::min(picked->y1, picked->y2),
        std::max(1, std::abs(picked->x2 - picked->x1)),
        std::max(1, std::abs(picked->y2 - picked->y1))
      );
      box &= cv::Rect2f(0, 0, (float)IMG_WIDTH, (float)IMG_HEIGHT);
      if (box.area() <= 1.f) { ++frame_id; continue; }
      
      const cv::Rect2f box_raw =
          unletterboxRect(box, IMG_WIDTH, IMG_HEIGHT, YOLO_INPUT_W, YOLO_INPUT_H);
      const double cx = box_raw.x + 0.5 * box_raw.width;
      const double target_x = 0.5 * IMG_WIDTH;
      const double e_px = target_x - cx;   // +e means object is to the left of target -> rotate right
      // 3) Convert to yaw (deg), then map to servo angle (0..180; 90 = forward)
      const double yaw_deg = pixelErrorToYawDeg(e_px, (double)IMG_WIDTH, hfov_deg);
      double servo = 90.0 + yaw_deg;
      servo_deg_ = std::clamp(servo, 0.0, 180.0);

      RCLCPP_INFO(this->get_logger(), "servo angle is: %.2f°", servo_deg_);
      publishState(static_cast<float>(servo_deg_));

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
        RCLCPP_INFO(this->get_logger(), "[px2] DETECT→TRACK (tracker=%s, servo=%d°)",
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
        const cv::Rect2f track_box_raw =
          unletterboxRect(cur, IMG_WIDTH, IMG_HEIGHT, YOLO_INPUT_W, YOLO_INPUT_H);
        const double cx = track_box_raw.x + 0.5 * track_box_raw.width;
        const double target_x = 0.5 * IMG_WIDTH;
        const double e_px = target_x - cx;   // +e means object is to the left of target -> rotate right
        // 3) Convert to yaw (deg), then map to servo angle (0..180; 90 = forward)
        const double yaw_deg = pixelErrorToYawDeg(e_px, (double)IMG_WIDTH, hfov_deg);
        double servo = 90.0 + yaw_deg;
        servo_deg_ = std::clamp(servo, 0.0, 180.0);

        RCLCPP_INFO(this->get_logger(), "servo angle is: %.2f°", servo_deg_);
        publishState(static_cast<float>(servo_deg_));
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
        
  if (!wait_for_ack_ms(ARK_TIME)) {   // tune timeout
    RCLCPP_WARN(get_logger(), "px3 ACK timeout; continuing");
  }
}

}  // namespace onnx_inference

int main(int argc, char * argv[])
{
  rclcpp::NodeOptions options;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<onnx_inference::OnnxInferenceNode>();
  rclcpp::ExecutorOptions opts;
  rclcpp::executors::MultiThreadedExecutor exec(opts, /*number_of_threads=*/4);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}

