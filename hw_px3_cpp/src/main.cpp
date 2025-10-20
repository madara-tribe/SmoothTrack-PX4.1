#include "hw_px3_cpp/px3_node.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<AngleForwarder>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    fprintf(stderr, "px3: fatal error: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}

