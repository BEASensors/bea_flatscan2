#pragma once

#include <mutex>

#include "common/parser.h"
#include "common/protocol.h"
#include "common/serial_port.h"

namespace bea_sensors {

class Flatscan : public rclcpp::Node {
 public:
  Flatscan();
  ~Flatscan();

  void SpinOnce();

 private:
  bool Initialize();
  bool InitializeConfiguration(const msg::Parameters& parameters);
  bool HandleConfiguration(const std::shared_ptr<srv::Configure::Request> req, std::shared_ptr<srv::Configure::Response> res);
  void SendMessage(const uint16_t& command, const uint16_t& data_length, const uint8_t* data);
  void HandleReceivedData(char* data, int length);
  void ParserRoutine();

 private:
  bool message_sent_ = false;
  std::mutex mutex_;

  rclcpp::Time last_scan_stamp_ = this->get_clock()->now();
  rclcpp::Time last_heartbeat_stamp_ = this->get_clock()->now();
  rclcpp::Time last_emergency_stamp_ = this->get_clock()->now();

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;
  rclcpp::Publisher<msg::Emergency>::SharedPtr emergency_publisher_;
  rclcpp::Publisher<msg::Heartbeat>::SharedPtr heartbeat_publisher_;
  rclcpp::Service<srv::Configure>::SharedPtr configuration_server_;

  SerialPort<Flatscan> com_;
  Protocol protocol_;
  Parser parser_;
};

}  // namespace bea_sensors
