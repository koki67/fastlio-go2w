#include <cstdint>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "fastlio_go2w_hesai/pointcloud_adapter.hpp"

namespace fastlio_go2w_hesai
{
namespace
{

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

template<typename T>
std::string as_string(const T & value)
{
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

}  // namespace

class HesaiPointcloudAdapterNode : public rclcpp::Node
{
public:
  HesaiPointcloudAdapterNode()
  : Node("hesai_pointcloud_adapter"), adapter_(read_config())
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/points_raw");
    const auto output_topic =
      declare_parameter<std::string>("output_topic", "/points_raw_fastlio");
    const auto diagnostics_topic = declare_parameter<std::string>(
      "diagnostics_topic", "/fastlio_go2w_hesai/diagnostics");

    output_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::SensorDataQoS().keep_last(100));
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic,
      rclcpp::QoS(10).reliable().transient_local());
    input_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::QoS(rclcpp::KeepLast(100)).reliable().durability_volatile(),
      std::bind(&HesaiPointcloudAdapterNode::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Adapting %s -> %s with lidar_time_offset_sec=%+.6f",
      input_topic.c_str(), output_topic.c_str(), adapter_.config().lidar_time_offset_sec);
  }

private:
  AdapterConfig read_config()
  {
    AdapterConfig config;
    config.lidar_time_offset_sec =
      declare_parameter<double>("lidar_time_offset_sec", 0.0);
    config.max_point_header_delta_sec =
      declare_parameter<double>("max_point_header_delta_sec", 0.2);
    const auto ring_count = declare_parameter<std::int64_t>("ring_count", 16);
    const auto minimum_points = declare_parameter<std::int64_t>("minimum_points", 10);
    if (ring_count <= 0 || ring_count > 65535) {
      throw std::invalid_argument("ring_count must be in [1, 65535]");
    }
    if (minimum_points <= 0) {
      throw std::invalid_argument("minimum_points must be positive");
    }
    config.ring_count = static_cast<std::uint16_t>(ring_count);
    config.minimum_points = static_cast<std::size_t>(minimum_points);
    return config;
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    const auto result = adapter_.convert(*message);
    if (result.converted) {
      output_->publish(result.cloud);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Dropped XT16 frame: %s",
        result.drop_reason.c_str());
    }
    publish_diagnostics(result.drop_reason);
  }

  void publish_diagnostics(const std::string & latest_drop_reason)
  {
    const auto & stats = adapter_.stats();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "fastlio_go2w_hesai/pointcloud_adapter";
    status.hardware_id = "recorded_pandar_xt16";
    status.level = latest_drop_reason.empty() ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = latest_drop_reason.empty() ? "converted" : "frame_dropped";
    status.values = {
      key_value("received_frames", as_string(stats.received_frames)),
      key_value("converted_frames", as_string(stats.converted_frames)),
      key_value("dropped_schema", as_string(stats.dropped_schema)),
      key_value("dropped_layout", as_string(stats.dropped_layout)),
      key_value("dropped_header_regression", as_string(stats.dropped_header_regression)),
      key_value("dropped_invalid_header", as_string(stats.dropped_invalid_header)),
      key_value("dropped_nonfinite_timestamp", as_string(stats.dropped_nonfinite_timestamp)),
      key_value(
        "dropped_timestamp_out_of_range", as_string(stats.dropped_timestamp_out_of_range)),
      key_value("dropped_too_few_points", as_string(stats.dropped_too_few_points)),
      key_value("invalid_points", as_string(stats.invalid_points)),
      key_value("latest_scan_width_sec", as_string(stats.latest_scan_width_sec)),
      key_value(
        "lidar_time_offset_sec", as_string(adapter_.config().lidar_time_offset_sec)),
      key_value("latest_drop_reason", latest_drop_reason)};

    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    message.status.push_back(std::move(status));
    diagnostics_->publish(message);
  }

  PointcloudAdapter adapter_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_;
};

}  // namespace fastlio_go2w_hesai

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fastlio_go2w_hesai::HesaiPointcloudAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
