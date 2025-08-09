// inference.cpp
#include <rclcpp/rclcpp.hpp>
#include "custom_msgs/msg/abs_result.hpp"
#include <chrono>

using namespace std::chrono_literals;

class AnglePublisherNode : public rclcpp::Node {
public:
  AnglePublisherNode()
  : Node("px2"),
    start_angle_(declare_parameter<int>("start_angle", 0)),
    end_angle_(declare_parameter<int>("end_angle", 179)),     // 180 messages (0..179)
    step_(declare_parameter<int>("step", 1)),
    publish_hz_(declare_parameter<double>("publish_hz", 20.0)),
    required_subs_(declare_parameter<int>("required_subscribers", 1))
  {
    if (step_ <= 0) step_ = 1;

    rclcpp::QoS qos(100);
    qos.reliable().durability_volatile();
    publisher_ = create_publisher<custom_msgs::msg::AbsResult>("inference", qos);

    // Wait for subscriber(s) without blocking forever
    auto t0 = now();
    auto deadline = t0 + rclcpp::Duration::from_seconds(5.0);
    while (rclcpp::ok() && publisher_->get_subscription_count() < static_cast<size_t>(required_subs_)) {
      if (now() > deadline) break;
      rclcpp::sleep_for(100ms);
    }
    RCLCPP_INFO(get_logger(),
                "px2: subscribers=%zu (required=%d) → starting publishing",
                publisher_->get_subscription_count(), required_subs_);

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
    RCLCPP_INFO(get_logger(),
      "px2: publishing %d angles [%d..%d] step %d at %.1f Hz",
      total_steps_, start_angle_, end_angle_, step_, publish_hz_);
  }

private:
  void onTimer() {
    if (published_ >= total_steps_) {
      RCLCPP_INFO(get_logger(), "px2: done publishing %d messages. Shutting down.", published_);
      rclcpp::shutdown();
      return;
    }
    custom_msgs::msg::AbsResult msg;
    msg.x_angle = static_cast<double>(current_angle_);
    publisher_->publish(msg);
    current_angle_ += step_;
    published_++;
  }

  rclcpp::Publisher<custom_msgs::msg::AbsResult>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int start_angle_, end_angle_, step_;
  double publish_hz_;
  int required_subs_;
  int current_angle_{0};
  int total_steps_{0};
  int published_{0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AnglePublisherNode>());
  rclcpp::shutdown();
  return 0;
}

