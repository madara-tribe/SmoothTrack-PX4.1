#ifndef HW_PX3_CPP_PX3_NODE_H
#define HW_PX3_CPP_PX3_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <custom_msgs/msg/abs_result.hpp>

#include <atomic>
#include <chrono>
#include <string>
#include <termios.h>  // speed_t

class AngleForwarder : public rclcpp::Node {
public:
  AngleForwarder();
  ~AngleForwarder() override;

private:
  // Timer tick: apply deadband + slew + clamp, then write to serial
  void on_timer_();

  // Serial helpers
  bool open_serial_(const std::string& port, int baud);
  bool send_angle_(int angle_deg);
  static speed_t baud_to_speed_(int baud);
  static std::chrono::steady_clock::time_point now_steady_();

  // --- Members ---
  int fd_;
  std::string serial_port_;
  int baud_;
  bool invert_;
  double center_deg_;
  double min_deg_;
  double max_deg_;
  double deadband_deg_;
  double max_slew_dps_;
  double write_hz_;

  std::atomic<double> target_angle_deg_;
  double last_cmd_angle_deg_;
  bool has_last_;
  std::chrono::steady_clock::time_point last_write_tp_;

  rclcpp::Subscription<custom_msgs::msg::AbsResult>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif // HW_PX3_CPP_PX3_NODE_H

