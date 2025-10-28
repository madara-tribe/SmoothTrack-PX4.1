#include "inference.h"

#define ONNX_YOLO_PATH "/weights/yolov7Tiny_640_640.onnx"
#define YOLO_INPUT_H 640
#define YOLO_INPUT_W 640
#define IMG_WIDTH 960
#define IMG_HEIGHT 540
#define ARK_TIME 300

using namespace std::chrono_literals;
static sort::SortTracker g_sorter;

namespace onnx_inference
{
OnnxInferenceNode::OnnxInferenceNode()
: Node("px2")
{
  // ----- Params -----
  this->declare_parameter<std::string>("device_path", "/dev/video2");
  this->declare_parameter<int>("lost_max_frames", 15);
  this->declare_parameter<bool>("save_frames", false);
  this->declare_parameter<bool>("enforce_bgr8", true);
  this->declare_parameter<std::string>("thirds_target_", "center");
  this->declare_parameter<bool>("preprocess_enable", true);
  
  device_path       = this->get_parameter("device_path").as_string();
  lost_max_frames_  = this->get_parameter("lost_max_frames").as_int();
  save_frames_      = this->get_parameter("save_frames").as_bool();
  enforce_bgr8_     = this->get_parameter("enforce_bgr8").as_bool();
  thirds_target_     = parseThirdsTarget(this->get_parameter("thirds_target_").as_string());
  preproc_enable_ = this->get_parameter("preprocess_enable").as_bool();  
  // ----- Publisher for servo setpoints -----
  rclcpp::QoS qos(10);
  qos.reliable().durability_volatile(); // stream; don"t latch servo commands
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

void onnx_inference::OnnxInferenceNode::saveThirdsOverlayIfNeeded(const cv::Mat& frame,
                                                                  const cv::Rect2d& roi_img,
                                                                  int frame_id,
                                                                  double target_x,
                                                                  const std::string& name_prefix)
{
  if (!this->save_frames_) return;
  cv::Mat vis = frame.clone();
  const int H = vis.rows;
  // draw thirds grid + target marker
  drawThirdsOverlay(vis, /*grid*/ {0,255,255}, /*mark*/ {0,0,255},
                    static_cast<int>(std::lround(target_x)), H/2);
  // draw ROI
  cv::rectangle(vis, roi_img, {0,255,0}, 2);
  // save
  const std::string out = this->pkg_path + "data/" + name_prefix
                        + std::to_string(frame_id) + ".png";
  cv::imwrite(out, vis);
}

void OnnxInferenceNode::callbackInference()
{
  // ensure we start only once, and only after px3_ready flips us on
  if (started_ || !px3_ready_) return;
  started_ = true;

  RCLCPP_INFO(this->get_logger(), "[px2] Starting DETECT→TRACK pipeline.");

  // Prepare tracker runtime
  int frame_id = 0;
  cv::Mat frame;
  
  while (rclcpp::ok() && cap_.read(frame)) {
    if (frame.empty()) continue;
    const cv::Size imsz = frame.size();
    
    //if (enforce_bgr8_){
    //  cv::Mat frame = to_bgr8(frame, enforce_bgr8_);}
    //cv::Mat frame_ = frame;
    if (preproc_enable_) {
      applyGlobalPreproc(frame, gamma_all, clahe_all, sharp_all, frame);
    }
    // YOLO until target appears
    cv::Mat inputImage = yolo_->preprocess(frame, YOLO_INPUT_H, YOLO_INPUT_W);
    std::vector<Ort::Value> outputTensors = yolo_->RunInference(inputImage);
    std::vector<Result> results = yolo_->postprocess(imsz, outputTensors);

    /*SORT_INTEGRATION_START*/
    // Convert YOLO results -> sort::Detection
    std::vector<sort::Detection> dets;
    dets.reserve(results.size());
    for (const auto& r : results) {
      cv::Rect2f b((float)std::min(r.x1, r.x2),
                  (float)std::min(r.y1, r.y2),
                  (float)std::max(1, std::abs(r.x2 - r.x1)),
                  (float)std::max(1, std::abs(r.y2 - r.y1)));
      // Use available fields: 'accuracy' (score) and 'obj_id' (class id)
      dets.push_back(sort::Detection{b, r.accuracy, r.obj_id});
    }
    // Update SORT and fetch tracks
    auto tracks = g_sorter.update(dets);

    (void)tracks; // currently not used for downstream logic here
    /*SORT_INTEGRATION_END*/
    std::optional<Result> picked;
    int best_area = -1;
    for (const auto& r : results) {
      int w = std::abs(r.x2 - r.x1);
      int h = std::abs(r.y2 - r.y1);
      int area = w * h;
      if (area > best_area) { best_area = area; picked = r; }
    }
    
    if (picked){
      // bbox sanity
      cv::Rect2d box(
        std::min(picked->x1, picked->x2),
        std::min(picked->y1, picked->y2),
        std::max(1, std::abs(picked->x2 - picked->x1)),
        std::max(1, std::abs(picked->y2 - picked->y1))
      );
      box &= cv::Rect2d(0, 0, (float)YOLO_INPUT_W, (float)YOLO_INPUT_H);
      if (box.area() <= 1.f) { ++frame_id; continue; }
      const int W = frame.cols, H = frame.rows;
      const cv::Rect2d roi_img =
          unletterboxRect2d(box, W, H, YOLO_INPUT_W, YOLO_INPUT_H);
      const double cx = roi_img.x + 0.5 * roi_img.width;
      double target_x = thirdsX(W, thirds_target_, cx);
      const double e_px = target_x - cx;
      // 3) Convert to yaw (deg), then map to servo angle (0..180; 90 = forward)
      const double yaw_deg = pixelErrorToYawDeg(e_px, (double)W, hfov_deg);
      double servo = 90.0 + yaw_deg;
      servo_deg_ = std::clamp(servo, 0.0, 180.0);

      //RCLCPP_INFO(this->get_logger(),
      //"[thirds] W=%d | bbox_cx=%.1f -> target_x=%.1f | e_px=%.1f | yaw=%.2f° | servo=%.2f°",
      //W, cx, target_x, e_px, yaw_deg, servo_deg_);

      publishState(static_cast<float>(servo_deg_));
      saveThirdsOverlayIfNeeded(frame, roi_img, frame_id, target_x, "detect_thirds_"); 
    } else {
      // 1) Take the best predicted track from SORT
      const auto& all = g_sorter.all_tracks(); // contains predicted boxes (KF.predict() already ran in update)
      const sort::Track* best = nullptr;

      // Heuristic: prefer high 'hits' (well-established), then smaller 'time_since_update' (recent)
      for (const auto& t : all) {
        if (!best ||
            t.hits > best->hits ||
            (t.hits == best->hits && t.time_since_update < best->time_since_update)) {
          best = &t;
        }
      }

      // Gate: only trust predictions that are not too stale
      constexpr int AGE_GATE = 5; // allow up to 5 frames without detection
      if (best && best->time_since_update <= AGE_GATE) {
        // 2) Use the predicted box for control (same coord system as YOLO input)
        cv::Rect2d pred_box = best->box;
        const int W = frame.cols, H = frame.rows;

        // If your pipeline uses letterbox undo, apply it to the predicted box too
        cv::Rect2d roi_img = unletterboxRect2d(pred_box, W, H, YOLO_INPUT_W, YOLO_INPUT_H);
        roi_img &= cv::Rect2d(0, 0, W, H);

        if (roi_img.area() > 1.0) {
          // 3) Compute servo command exactly like when you have a detection
          const double cx = roi_img.x + 0.5 * roi_img.width;
          const double target_x = thirdsX(W, thirds_target_, cx);
          const double e_px = target_x - cx;
          const double yaw_deg = pixelErrorToYawDeg(e_px, (double)W, hfov_deg);
          servo_deg_ = std::clamp(90.0 + yaw_deg, 0.0, 180.0);

          publishState(static_cast<float>(servo_deg_));
          // Optional: overlay to visualize prediction usage
          saveThirdsOverlayIfNeeded(frame, roi_img, frame_id, target_x, "pred_thirds_");
        }
      }
    }
    ++frame_id; 
  } 
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
