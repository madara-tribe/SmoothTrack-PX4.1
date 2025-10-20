#include "hw_px3_cpp/px3_node.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <thread>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>

using namespace std::chrono_literals;

AngleForwarder::AngleForwarder()
  : rclcpp::Node("px3"),
    fd_(-1),
    target_angle_deg_(90.0),
    last_cmd_angle_deg_(90.0),
    has_last_(false)
{
  // ---- Parameters (mirrors Python main.py) ----
  serial_port_  = this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
  baud_         = this->declare_parameter<int>("baud", 9600);        // faster default
  invert_       = this->declare_parameter<bool>("invert_angle", true);
  center_deg_   = this->declare_parameter<double>("center_deg", 90.0);
  min_deg_      = this->declare_parameter<double>("min_deg", 0.0);
  max_deg_      = this->declare_parameter<double>("max_deg", 180.0);
  deadband_deg_ = this->declare_parameter<double>("deadband_deg", 0.6);
  max_slew_dps_ = this->declare_parameter<double>("max_slew_deg_per_s", 180.0);
  write_hz_     = this->declare_parameter<double>("write_hz", 200.0);  // high-rate loop

  // ---- Serial open & center ----
  if (!open_serial_(serial_port_, baud_)) {
    RCLCPP_FATAL(get_logger(), "Failed to open serial port %s @ %d", serial_port_.c_str(), baud_);
    throw std::runtime_error("serial open failed");
  }
  // Arduino auto-reset delay
  std::this_thread::sleep_for(std::chrono::seconds(2));

  last_cmd_angle_deg_ = center_deg_;
  target_angle_deg_.store(center_deg_, std::memory_order_relaxed);
  send_angle_(static_cast<int>(std::lround(center_deg_)));
  has_last_ = true;

  RCLCPP_INFO(get_logger(), "px3: opened serial %s @ %d; centered to %.1f°",
              serial_port_.c_str(), baud_, center_deg_);

  // ---- Publish px3_ready (latched / TRANSIENT_LOCAL) ----
  rclcpp::QoS ready_qos(1);
  ready_qos.transient_local().reliable();
  ready_pub_ = this->create_publisher<std_msgs::msg::Bool>("px3_ready", ready_qos);
  std_msgs::msg::Bool ready;
  ready.data = true;
  ready_pub_->publish(ready);
  RCLCPP_INFO(get_logger(), "px3: published px3_ready=True");

  // ---- Subscribe to setpoints (AbsResult) ----
  sub_ = this->create_subscription<custom_msgs::msg::AbsResult>(
    "inference", rclcpp::SensorDataQoS(),
    [this](custom_msgs::msg::AbsResult::ConstSharedPtr msg) {
      double ang = static_cast<double>(msg->x_angle);
      if (invert_) ang = 180.0 - ang;
      // clamp target immediately
      if (ang < min_deg_) ang = min_deg_;
      if (ang > max_deg_) ang = max_deg_;
      target_angle_deg_.store(ang, std::memory_order_relaxed);
    });

  // ---- Timer: rate-limited writes, last-value-wins ----
  const double period_s = (write_hz_ > 1.0) ? (1.0 / write_hz_) : 0.01;
  last_write_tp_ = now_steady_();
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(period_s)),
    [this]() { this->on_timer_(); });
}

AngleForwarder::~AngleForwarder() {
  if (fd_ >= 0) ::close(fd_);
}

void AngleForwarder::on_timer_() {
  const auto now = now_steady_();
  double dt = std::chrono::duration<double>(now - last_write_tp_).count();
  if (dt <= 0.0) dt = 1e-3;
  if (dt > 0.05) dt = 0.05; // cap to avoid big jumps after pauses

  double tgt = target_angle_deg_.load(std::memory_order_relaxed);
  double cmd = tgt;

  if (has_last_) {
    double delta = tgt - last_cmd_angle_deg_;
    if (std::fabs(delta) < deadband_deg_) {
      return; // within deadband; skip write
    }
    double step_max = std::max(0.0, max_slew_dps_) * dt;
    if (step_max > 0.0) {
      if (delta >  step_max) delta =  step_max;
      if (delta < -step_max) delta = -step_max;
    }
    cmd = last_cmd_angle_deg_ + delta;
  } else {
    has_last_ = true;
  }

  // final clamp
  if (cmd < min_deg_) cmd = min_deg_;
  if (cmd > max_deg_) cmd = max_deg_;

  // quantize to integer degrees for Arduino
  int cmd_i = static_cast<int>(std::lround(cmd));
  if (!send_angle_(cmd_i)) {
    RCLCPP_WARN(get_logger(), "serial write failed (continuing)");
    return;
  }
  last_cmd_angle_deg_ = cmd;
  last_write_tp_ = now;
}

bool AngleForwarder::send_angle_(int angle_deg) {
  // Simple ASCII protocol: "<angle>\n"
  char buf[16];
  int n = std::snprintf(buf, sizeof(buf), "%d\n", angle_deg);
  if (n <= 0) return false;
  ssize_t w = ::write(fd_, buf, static_cast<size_t>(n));
  if (w != n) return false;
  // Optionally: tcdrain(fd_);
  return true;
}

speed_t AngleForwarder::baud_to_speed_(int baud) {
  switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    default:     return B115200;
  }
}

bool AngleForwarder::open_serial_(const std::string& port, int baud) {
  int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    RCLCPP_ERROR(get_logger(), "open(%s) failed: %s", port.c_str(), std::strerror(errno));
    return false;
  }

  // Clear O_NONBLOCK to do blocking writes (short and deterministic)
  int flags = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

  struct termios tio{};
  if (tcgetattr(fd, &tio) != 0) {
    RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", std::strerror(errno));
    ::close(fd);
    return false;
  }

  cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CSTOPB;   // 1 stop bit
  tio.c_cflag &= ~CRTSCTS;  // no HW flow
  tio.c_cflag &= ~PARENB;   // no parity
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;       // 8 data bits

  speed_t spd = baud_to_speed_(baud);
  cfsetispeed(&tio, spd);
  cfsetospeed(&tio, spd);

  tio.c_cc[VMIN]  = 0;
  tio.c_cc[VTIME] = 1;      // read timeout 0.1s

  if (tcsetattr(fd, TCSANOW, &tio) != 0) {
    RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", std::strerror(errno));
    ::close(fd);
    return false;
  }
  tcflush(fd, TCIOFLUSH);

  fd_ = fd;
  return true;
}

std::chrono::steady_clock::time_point AngleForwarder::now_steady_() {
  return std::chrono::steady_clock::now();
}

