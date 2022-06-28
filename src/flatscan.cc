#include "flatscan.h"

#include <angles/angles.h>

#include <thread>

namespace bea_sensors {

Flatscan::Flatscan() : Node("bea_flatscan") { Initialize(); }

Flatscan::~Flatscan() { com_.Close(); }

void Flatscan::SpinOnce() {
  const rclcpp::Time current_scan_stamp{parser_.laser_scan().header.stamp};
  if (laser_scan_publisher_->get_subscription_count() > 0 && last_scan_stamp_ < current_scan_stamp) {
    laser_scan_publisher_->publish(parser_.laser_scan());
    last_scan_stamp_ = current_scan_stamp;
  }

  const rclcpp::Time current_heartbeat_stamp{parser_.heartbeat().header.stamp};
  if (heartbeat_publisher_->get_subscription_count() > 0 && last_heartbeat_stamp_ < current_heartbeat_stamp) {
    heartbeat_publisher_->publish(parser_.heartbeat());
    last_heartbeat_stamp_ = current_heartbeat_stamp;
  }

  const rclcpp::Time current_emergency_stamp{parser_.emergency().header.stamp};
  if (emergency_publisher_->get_subscription_count() > 0 && last_emergency_stamp_ < current_emergency_stamp) {
    emergency_publisher_->publish(parser_.emergency());
    last_emergency_stamp_ = current_emergency_stamp;
  }
}

bool Flatscan::Initialize() {
  std::string port;
  this->get_parameter_or("port", port, std::string("/dev/ttyUSB0"));
  int baudrate;
  this->get_parameter_or("baudrate", baudrate, 921600);

  std::string scan_topic, heartbeat_topic, emergency_topic;
  this->get_parameter_or("scan_topic", scan_topic, std::string("/scan"));
  this->get_parameter_or("heartbeat_topic", heartbeat_topic, std::string("/heartbeat"));
  this->get_parameter_or("emergency_topic", emergency_topic, std::string("/emergency"));

  msg::Parameters parameters;
  this->get_parameter_or("scan_frame_id", parameters.header.frame_id, std::string("laser_link"));
  this->get_parameter_or("min_range", parameters.range_min, static_cast<float>(0.));
  this->get_parameter_or("max_range", parameters.range_max, static_cast<float>(8.));

  int temp;
  this->get_parameter_or("enable_temperature", temp, static_cast<int>(1));
  parameters.temperature = static_cast<uint8_t>(temp);
  this->get_parameter_or("information_in_mdi", temp, static_cast<int>(0));
  parameters.information = static_cast<uint8_t>(temp);
  std::string detection_field_mode;
  this->get_parameter_or("detection_field_mode", detection_field_mode, std::string("HD"));
  if (detection_field_mode == "HD") {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Flatscan works in HD mode, loading HD configurations");
    parameters.mode = 1;
    parameters.number_of_spots = 400;
  } else if (detection_field_mode == "HS") {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Flatscan works in HS mode, loading HS configurations");
    parameters.mode = 0;
    parameters.number_of_spots = 100;
  } else {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Flatscan works in %s(UNKNOWN) mode, loading HD configurations by default", detection_field_mode.c_str());
    parameters.mode = 1;
    parameters.number_of_spots = 400;
  }
  this->get_parameter_or("optimization", temp, static_cast<int>(0));
  parameters.optimization = static_cast<uint8_t>(temp);
  float angle_first, angle_last;
  this->get_parameter_or("angle_first", angle_first, static_cast<float>(0.));
  this->get_parameter_or("angle_last", angle_last, static_cast<float>(108.));
  parameters.angle_first = static_cast<uint16_t>(angle_first * 1e2);
  parameters.angle_last = static_cast<uint16_t>(angle_last * 1e2);
  this->get_parameter_or("enable_counter", temp, static_cast<int>(1));
  parameters.counter = static_cast<uint8_t>(temp);
  this->get_parameter_or("heartbeat_period", temp, static_cast<int>(5));
  parameters.heartbeat_period = static_cast<uint8_t>(temp);
  this->get_parameter_or("enable_facet", temp, static_cast<int>(1));
  parameters.facet = static_cast<uint8_t>(temp);
  this->get_parameter_or("averaging_setting", temp, static_cast<int>(0));
  parameters.averaging = static_cast<uint8_t>(temp);

  laser_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, 1);
  heartbeat_publisher_ = this->create_publisher<msg::Heartbeat>(heartbeat_topic, 1);
  emergency_publisher_ = this->create_publisher<msg::Emergency>(emergency_topic, 1);
  configuration_server_ = this->create_service<srv::Configure>(
      "configure", std::bind(&Flatscan::HandleConfiguration, this, std::placeholders::_1, std::placeholders::_2));

  com_.RegisterCallback(this, &Flatscan::HandleReceivedData);
  com_.Connect(port, baudrate);

  std::thread thread(&Flatscan::ParserRoutine, this);
  thread.detach();

  InitializeConfiguration(parameters);
  return true;
}

bool Flatscan::InitializeConfiguration(const msg::Parameters& parameters) {
  parser_.Initialize(parameters);
  auto req{std::make_shared<srv::Configure::Request>()};
  req->command = "set_parameters";
  req->subcommand = "";
  auto res{std::make_shared<srv::Configure::Response>()};
  HandleConfiguration(req, res);
  return res->success;
}

bool Flatscan::HandleConfiguration(const std::shared_ptr<srv::Configure::Request> req, std::shared_ptr<srv::Configure::Response> res) {
  DataFrame frame;
  bool success{false};
  if (!parser_.GenerateDataFrame(req->command, req->subcommand, req->value, success, res->description, frame)) {
    res->success = success;
    return true;
  }
  res->success = success;
  SendMessage(frame.command(), frame.length(), frame.data());
  return true;
}

void Flatscan::SendMessage(const uint16_t& command, const uint16_t& data_length, const uint8_t* data) {
  std::unique_lock<std::mutex> lock(mutex_);
  message_sent_ = false;
  lock.unlock();
  uint8_t data_out[data_length + kFrameMinimalLength];
  uint16_t length = protocol_.GenerateRawFrame(command, data, data_length, data_out);
  if (length < kFrameMinimalLength) {
    return;
  }

  uint8_t retries{0};
  while (!message_sent_ && retries < 10) {
    com_.Write((char*)data_out, length);
    ++retries;
    sleep(1);
  }

  if (!message_sent_) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "send message failed");
  }
}

void Flatscan::HandleReceivedData(char* data, int length) {
  for (int i = 0; i < length; ++i) {
    if (protocol_.InsertByte(data[i]) < 0) {
      continue;
    }
  }
}

void Flatscan::ParserRoutine() {
  while (true) {
    DataFrame frame;
    if (!protocol_.GetLatestDataFrame(frame)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    if (!parser_.ParseDataFrame(frame)) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Parse frame failed");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));

      continue;
    }

    if (frame.command() != MDI) {
      std::unique_lock<std::mutex> lock(mutex_);
      message_sent_ = true;
      lock.unlock();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

}  // namespace bea_sensors
