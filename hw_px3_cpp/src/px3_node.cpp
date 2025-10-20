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

// ---- Firmata/Telemetrix minimal constants ----
// Ref: StandardFirmata / Telemetrix command set (subset)
namespace firmata {
  constexpr uint8_t START_SYSEX        = 0xF0;
  constexpr uint8_t END_SYSEX          = 0xF7;
  constexpr uint8_t SET_PIN_MODE       = 0xF4;   // [pin] [mode]
  constexpr uint8_t SERVO_CONFIG       = 0x70;   // sysex: [pin LSB][pin MSB][min LSB][min MSB][max LSB][max MSB]
  constexpr uint8_t EXTENDED_ANALOG    = 0x6F;   // sysex: [pin][value LSB][value MSB]...
  // Pin modes (per Firmata)
  constexpr uint8_t PIN_MODE_SERVO     = 0x04;
}

// Servo pin must match Python (Telemetrix used 9)
static constexpr int SERVO_PIN = 9;

static inline uint8_t lsb7(int v) { return static_cast<uint8_t>(v & 0x7F); }
static inline uint8_t msb7(int v) { return static_cast<uint8_t>((v >> 7) & 0x7F); }

AngleForwarder::AngleForwarder()
  : rclcpp::Node("px3"),
    fd_(-1),
    target_angle_deg_(90.0),
    last_cmd_angle_deg_(90.0),
    has_last_(false)
{
  // ---- Parameters (mirrors Python main.py + extras) ----
  serial_port_  = this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
  baud_         = this->declare_parameter<int>("baud", 9600);            // Python file shows 9600 default. Change if your .ino differs. 
  invert_       = this->declare_parameter<bool>("invert_angle", true);
  center_deg_   = this->declare_parameter<double>("center_deg", 90.0);
  min_deg_      = this->declare_parameter<double>("min_deg", 0.0);
  max_deg_      = this->declare_parameter<double>("max_deg", 180.0);
  deadband_deg_ = this->declare_parameter<double>("deadband_deg", 0.6);
  max_slew_dps_ = this->declare_parameter<double>("max_slew_deg_per_s", 180.0);
  write_hz_     = this->declare_parameter<double>("write_hz", 200.0);

  // ---- Serial open ----
  if (!open_serial_(serial_port_, baud_)) {
    RCLCPP_FATAL(get_logger(), "Failed to open serial port %s @ %d", serial_port_.c_str(), baud_);
    throw std::runtime_error("serial open failed");
  }

  // Arduino auto-reset delay
  std::this_thread::sleep_for(2s);

  // ---- Tell Telemetrix/Firmata: SERVO mode and (optionally) servo range ----
  // 1) Pin mode -> SERVO
  {
    uint8_t pkt[3] = { firmata::SET_PIN_MODE, static_cast<uint8_t>(SERVO_PIN), firmata::PIN_MODE_SERVO };
    ::write(fd_, pkt, sizeof(pkt));
  }
  // 2) Servo config (optional but good): min=544, max=2400 (us) — matches Telemetrix typical defaults
  {
    uint8_t msg[] = {
      firmata::START_SYSEX,
      firmata::SERVO_CONFIG,
      lsb7(SERVO_PIN), msb7(SERVO_PIN),
      lsb7(544),       msb7(544),
      lsb7(2400),      msb7(2400),
      firmata::END_SYSEX
    };
    ::write(fd_, msg, sizeof(msg));
  }

  // ---- Center the servo via Extended Analog (angle 0..180) ----
  last_cmd_angle_deg_ = center_deg_;
  target_angle_deg_.store(center_deg_, std::memory_order_relaxed);
  // Write once; ignore return (board may still be booting)
  (void)send_angle_(static_cast<int>(std::lround(center_deg_)));
  has_last_ = true;

  RCLCPP_INFO(get_logger(), "px3: opened serial %s @ %d; centered to %.1f° (Firmata/Telemetrix)",
              serial_port_.c_str(), baud_, center_deg_);

  // ---- Publish px3_ready (latched / TRANSIENT_LOCAL) ----
  rclcpp::QoS ready_qos(1);
  ready_qos.transient_local().reliable();
  ready_pub_ = this->create_publisher<std_msgs::msg::Bool>("px3_ready", ready_qos);
  std_msgs::msg::Bool ready; ready.data = true;
  ready_pub_->publish(ready);
  RCLCPP_INFO(get_logger(), "px3: published px3_ready=True");

  // ---- Subscribe to setpoints (AbsResult) ----
  sub_ = this->create_subscription<custom_msgs::msg::AbsResult>(
    "inference", rclcpp::SensorDataQoS(),
    [this](custom_msgs::msg::AbsResult::ConstSharedPtr msg) {
      double ang = static_cast<double>(msg->x_angle);
      if (invert_) ang = 180.0 - ang;
      ang = std::max(min_deg_, std::min(max_deg_, ang));
      target_angle_deg_.store(ang, std::memory_order_relaxed);
    });

  // ---- Timer: rate-limited writes, last-value-wins ----
  const double period_s = (write_hz_ > 1.0) ? (1.0 / write_hz_) : 0.01;
  last_write_tp_ = now_steady_();
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s)),
    [this]() { this->on_timer_(); });
}

AngleForwarder::~AngleForwarder() {
  if (fd_ >= 0) ::close(fd_);
}

void AngleForwarder::on_timer_() {
  const auto now = now_steady_();
  double dt = std::chrono::duration<double>(now - last_write_tp_).count();
  if (dt <= 0.0) dt = 1e-3;
  if (dt > 0.05) dt = 0.05;

  double tgt = target_angle_deg_.load(std::memory_order_relaxed);
  double cmd = tgt;

  if (has_last_) {
    double delta = tgt - last_cmd_angle_deg_;
    if (std::fabs(delta) < deadband_deg_) return;
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

  int cmd_i = static_cast<int>(std::lround(cmd));
  if (!send_angle_(cmd_i)) {
    RCLCPP_WARN(get_logger(), "serial write failed (continuing)");
    return;
  }
  last_cmd_angle_deg_ = cmd;
  last_write_tp_ = now;
}

// ---- Firmata/Telemetrix: write angle with EXTENDED_ANALOG ----
bool AngleForwarder::send_angle_(int angle_deg) {
  if (angle_deg < 0) angle_deg = 0;
  if (angle_deg > 180) angle_deg = 180;

  // Telemetrix accepts Firmata EXTENDED_ANALOG for analog/servo write on any pin:
  // [START_SYSEX, EXTENDED_ANALOG, pin, value LSB, value MSB, END_SYSEX]
  uint8_t msg[] = {
    firmata::START_SYSEX,
    firmata::EXTENDED_ANALOG,
    static_cast<uint8_t>(SERVO_PIN & 0x7F),
    lsb7(angle_deg),
    msb7(angle_deg),
    firmata::END_SYSEX
  };
  ssize_t w = ::write(fd_, msg, sizeof(msg));
  if (w != static_cast<ssize_t>(sizeof(msg))) return false;

  // Ensure bytes leave kernel buffer promptly during testing
  tcdrain(fd_);
  return true;
}

speed_t AngleForwarder::baud_to_speed_(int baud) {
  switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    default:     return B9600; // default to match your Python default
  }
}

bool AngleForwarder::open_serial_(const std::string& port, int baud) {
  int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    RCLCPP_ERROR(get_logger(), "open(%s) failed: %s", port.c_str(), std::strerror(errno));
    return false;
  }
  int flags = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

  struct termios tio{};
  if (tcgetattr(fd, &tio) != 0) {
    RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", std::strerror(errno));
    ::close(fd); return false;
  }

  cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CSTOPB;
  tio.c_cflag &= ~CRTSCTS;
  tio.c_cflag &= ~PARENB;
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;

  speed_t spd = baud_to_speed_(baud);
  cfsetispeed(&tio, spd);
  cfsetospeed(&tio, spd);

  tio.c_cc[VMIN]  = 0;
  tio.c_cc[VTIME] = 1;

  if (tcsetattr(fd, TCSANOW, &tio) != 0) {
    RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", std::strerror(errno));
    ::close(fd); return false;
  }
  tcflush(fd, TCIOFLUSH);

  fd_ = fd;
  return true;
}

std::chrono::steady_clock::time_point AngleForwarder::now_steady_() {
  return std::chrono::steady_clock::now();
}
