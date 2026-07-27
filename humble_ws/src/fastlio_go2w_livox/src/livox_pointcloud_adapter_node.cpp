#include <chrono>
#include <cmath>
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
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "fastlio_go2w_livox/pointcloud_adapter.hpp"

namespace fastlio_go2w_livox
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

class LivoxPointcloudAdapterNode : public rclcpp::Node
{
public:
  LivoxPointcloudAdapterNode()
  : Node("livox_pointcloud_adapter"), adapter_(read_config())
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/livox/lidar");
    const auto output_topic =
      declare_parameter<std::string>("output_topic", "/livox/lidar_fastlio");
    const auto diagnostics_topic = declare_parameter<std::string>(
      "diagnostics_topic", "/fastlio_go2w_livox/diagnostics");
    const auto diagnostics_period_sec =
      declare_parameter<double>("diagnostics_publish_period_sec", 1.0);
    if (!std::isfinite(diagnostics_period_sec) || diagnostics_period_sec <= 0.0) {
      throw std::invalid_argument("diagnostics_publish_period_sec must be finite and positive");
    }

    output_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(
      output_topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile());
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic, rclcpp::QoS(10).reliable().transient_local());
    input_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::QoS(rclcpp::KeepLast(256)).reliable().durability_volatile(),
      std::bind(&LivoxPointcloudAdapterNode::cloud_callback, this, std::placeholders::_1));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::duration<double>(diagnostics_period_sec),
      std::bind(&LivoxPointcloudAdapterNode::publish_diagnostics_if_changed, this));

    RCLCPP_INFO(
      get_logger(), "Adapting Livox PointCloud2 %s -> CustomMsg %s",
      input_topic.c_str(), output_topic.c_str());
  }

private:
  AdapterConfig read_config()
  {
    AdapterConfig config;
    config.max_point_header_delta_sec =
      declare_parameter<double>("max_point_header_delta_sec", 0.2);
    const auto minimum_points = declare_parameter<std::int64_t>("minimum_points", 10);
    if (minimum_points <= 0) {
      throw std::invalid_argument("minimum_points must be positive");
    }
    config.minimum_points = static_cast<std::size_t>(minimum_points);
    return config;
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    const auto result = adapter_.convert(*message);
    latest_drop_reason_ = result.drop_reason;
    diagnostics_dirty_ = true;
    if (result.converted) {
      output_->publish(result.message);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Dropped Livox PointCloud2 frame: %s",
        result.drop_reason.c_str());
    }
  }

  void publish_diagnostics_if_changed()
  {
    if (!diagnostics_dirty_) {
      return;
    }
    diagnostics_dirty_ = false;
    const auto & stats = adapter_.stats();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "fastlio_go2w_livox/pointcloud_adapter";
    status.hardware_id = "recorded_livox_mid360";
    status.level = latest_drop_reason_.empty() ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = latest_drop_reason_.empty() ? "converted" : "frame_dropped";
    status.values = {
      key_value("received_frames", as_string(stats.received_frames)),
      key_value("converted_frames", as_string(stats.converted_frames)),
      key_value("critical_drops", as_string(stats.critical_drops())),
      key_value("reordered_frames", as_string(stats.reordered_frames)),
      key_value(
        "quantization_clamped_frames", as_string(stats.quantization_clamped_frames)),
      key_value(
        "quantization_clamped_points", as_string(stats.quantization_clamped_points)),
      key_value("dropped_invalid_header", as_string(stats.dropped_invalid_header)),
      key_value("dropped_header_regression", as_string(stats.dropped_header_regression)),
      key_value("dropped_schema", as_string(stats.dropped_schema)),
      key_value("dropped_layout", as_string(stats.dropped_layout)),
      key_value(
        "dropped_nonfinite_coordinate", as_string(stats.dropped_nonfinite_coordinate)),
      key_value("dropped_nonfinite_intensity", as_string(stats.dropped_nonfinite_intensity)),
      key_value("dropped_nonfinite_timestamp", as_string(stats.dropped_nonfinite_timestamp)),
      key_value(
        "dropped_intensity_out_of_range", as_string(stats.dropped_intensity_out_of_range)),
      key_value(
        "dropped_intensity_nonintegral", as_string(stats.dropped_intensity_nonintegral)),
      key_value("dropped_negative_offset", as_string(stats.dropped_negative_offset)),
      key_value("dropped_offset_too_large", as_string(stats.dropped_offset_too_large)),
      key_value("dropped_too_few_points", as_string(stats.dropped_too_few_points)),
      key_value("latest_scan_width_sec", as_string(stats.latest_scan_width_sec)),
      key_value("latest_drop_reason", latest_drop_reason_)};

    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    message.status.push_back(std::move(status));
    diagnostics_->publish(message);
  }

  PointcloudAdapter adapter_;
  std::string latest_drop_reason_;
  bool diagnostics_dirty_{false};
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr output_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace fastlio_go2w_livox

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fastlio_go2w_livox::LivoxPointcloudAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
