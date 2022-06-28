#include "flatscan.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bea_sensors::Flatscan>());
  rclcpp::shutdown();

  return 0;
}
