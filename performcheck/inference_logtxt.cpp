// inference.cpp
#include <rclcpp/rclcpp.hpp>
#include "custom_msgs/msg/abs_result.hpp"
#include <chrono>
#include <algorithm>  // for std::min/std::max

using namespace std::chrono_literals;

class AnglePublisherNode : public rclcpp::Node {
public:
  AnglePublisherNode()
  : Node("px2"),
    start_angle_(declare_parameter<int>("start_angle", 0)),
    end_angle_(declare_parameter<int>("end_angle", 179)),     // 180 messages (0..179)
    step_(declare_parameter<int>("step", 1)),
    publish_hz_(declare_parameter<double>("publish_hz", 20.0)),
    required_subs_(declare_parameter<int>("required_subscribers", 1)),
    start_delay_ms_(declare_parameter<int>("start_delay_ms", 500)),  // extra settle time after subs found
    log_every_(declare_parameter<int>("log_every", 30)),
    wait_timeout_s_(declare_parameter<double>("wait_timeout_s", 5.0))
  {
    if (step_ <= 0) step_ = 1;

    // QoS: reliable, volatile, decent queue
    rclcpp::QoS qos(100);
    qos.reliable().durability_volatile();
    publisher_ = create_publisher<custom_msgs::msg::AbsResult>("inference", qos);

    // ---- Wait for subscriber(s) (bounded) ----
    const auto t_deadline = now() + rclcpp::Duration::from_seconds(wait_timeout_s_);
    while (rclcpp::ok() &&
           publisher_->get_subscription_count() < static_cast<size_t>(required_subs_)) {
      if (now() > t_deadline) break;
      rclcpp::sleep_for(100ms);
    }
    RCLCPP_INFO(get_logger(),
                "px2: subscribers=%zu (required=%d)",
                publisher_->get_subscription_count(), required_subs_);

    // Optional small delay to let px3 finish serial init, etc.
    if (start_delay_ms_ > 0) {
      RCLCPP_INFO(get_logger(), "px2: start_delay_ms=%d", start_delay_ms_);
      rclcpp::sleep_for(std::chrono::milliseconds(start_delay_ms_));
    }

    // Compute steps
    total_steps_ = (end_angle_ >= start_angle_)
                   ? ((end_angle_ - start_angle_) / step_ + 1)
                   : 0;

    // Timer period
    auto period_ms = std::chrono::milliseconds(
        std::max(1, static_cast<int>(1000.0 / std::max(1e-3, publish_hz_)))
    );
    timer_ = create_wall_timer(period_ms, std::bind(&AnglePublisherNode::onTimer, this));

    current_angle_ = start_angle_;

    // Init stats
    stats_started_ = false;
    sum_period_ms_ = 0.0;
    min_period_ms_ = 1e12;
    max_period_ms_ = 0.0;
    interval_count_ = 0;

    RCLCPP_INFO(get_logger(),
      "px2: publishing %d angles [%d..%d] step %d at %.1f Hz "
      "(period=%lld ms, log_every=%d)",
      total_steps_, start_angle_, end_angle_, step_, publish_hz_,
      static_cast<long long>(period_ms.count()), log_every_);
  }

private:
  using Clock = std::chrono::steady_clock;

  void onTimer() {
    if (published_ >= total_steps_) {
      // Final summary (only if we have at least one interval)
      if (interval_count_ > 0) {
        const double avg_ms = sum_period_ms_ / static_cast<double>(interval_count_);
        const double eff_hz = avg_ms > 0.0 ? 1000.0 / avg_ms : 0.0;
        RCLCPP_INFO(get_logger(),
          "px2: publish timing summary -> N=%d intervals | avg=%.3f ms "
          "(min=%.3f, max=%.3f) => ~%.2f Hz",
          interval_count_, avg_ms, min_period_ms_, max_period_ms_, eff_hz);
      }
      RCLCPP_INFO(get_logger(), "px2: done publishing %d messages. Shutting down.", published_);
      rclcpp::shutdown();
      return;
    }

    // ---- Measure publish→publish interval ----
    const auto now_tp = Clock::now();
    if (stats_started_) {
      const double dt_ms =
          std::chrono::duration<double, std::milli>(now_tp - last_pub_tp_).count();
      sum_period_ms_ += dt_ms;
      min_period_ms_ = std::min(min_period_ms_, dt_ms);
      max_period_ms_ = std::max(max_period_ms_, dt_ms);
      interval_count_++;

      if (log_every_ > 0 && (published_ % log_every_ == 0)) {
        const double avg_ms = sum_period_ms_ / static_cast<double>(interval_count_);
        const double eff_hz = avg_ms > 0.0 ? 1000.0 / avg_ms : 0.0;
        RCLCPP_INFO(get_logger(),
          "px2: sent=%d | last dt=%.3f ms | avg=%.3f ms (~%.2f Hz)",
          published_, dt_ms, avg_ms, eff_hz);
      }
    } else {
      stats_started_ = true;  // first timestamp captured after first publish
    }
    last_pub_tp_ = now_tp;

    // ---- Publish ----
    custom_msgs::msg::AbsResult msg;
    msg.x_angle = static_cast<double>(current_angle_);
    publisher_->publish(msg);

    current_angle_ += step_;
    published_++;
  }

  // ROS
  rclcpp::Publisher<custom_msgs::msg::AbsResult>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Params
  int start_angle_, end_angle_, step_;
  double publish_hz_;
  int required_subs_;
  int start_delay_ms_;
  int log_every_;
  double wait_timeout_s_;

  // State
  int current_angle_{0};
  int total_steps_{0};
  int published_{0};

  // Timing stats
  bool stats_started_{false};
  Clock::time_point last_pub_tp_;
  double sum_period_ms_{0.0};
  double min_period_ms_{0.0};
  double max_period_ms_{0.0};
  int interval_count_{0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AnglePublisherNode>());
  rclcpp::shutdown();
  return 0;
}

