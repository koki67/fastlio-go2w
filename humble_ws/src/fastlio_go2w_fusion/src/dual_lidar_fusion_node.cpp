// Copyright 2026 Koki Tanaka
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include "fastlio_go2w_fusion/fusion_core.hpp"
#include "fastlio_go2w_fusion/pointcloud_parser.hpp"

namespace fastlio_go2w_fusion
{
namespace
{

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using LivoxMessage = livox_ros_driver2::msg::CustomMsg;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

std::uint64_t stampToNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000U) {
    return 0U;
  }
  return static_cast<std::uint64_t>(stamp.sec) * 1000000000ULL + stamp.nanosec;
}

builtin_interfaces::msg::Time nanosecondsToStamp(std::uint64_t nanoseconds)
{
  builtin_interfaces::msg::Time stamp;
  const std::uint64_t seconds = nanoseconds / 1000000000ULL;
  if (seconds > static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max())) {
    throw std::overflow_error("timestamp cannot be represented by builtin_interfaces/Time");
  }
  stamp.sec = static_cast<std::int32_t>(seconds);
  stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000ULL);
  return stamp;
}

std::int64_t secondsToSignedNanoseconds(double seconds)
{
  if (!std::isfinite(seconds)) {
    throw std::invalid_argument("time offset must be finite");
  }
  const long double value = static_cast<long double>(seconds) * 1000000000.0L;
  if (value < static_cast<long double>(std::numeric_limits<std::int64_t>::min()) ||
    value > static_cast<long double>(std::numeric_limits<std::int64_t>::max()))
  {
    throw std::invalid_argument("time offset is outside int64 nanosecond range");
  }
  return static_cast<std::int64_t>(std::llround(value));
}

std::uint64_t secondsToUnsignedNanoseconds(double seconds)
{
  if (!std::isfinite(seconds) || seconds < 0.0) {
    throw std::invalid_argument("duration must be finite and non-negative");
  }
  const long double value = static_cast<long double>(seconds) * 1000000000.0L;
  if (value > static_cast<long double>(std::numeric_limits<std::uint64_t>::max())) {
    throw std::invalid_argument("duration is outside uint64 nanosecond range");
  }
  return static_cast<std::uint64_t>(value + 0.5L);
}

template<typename T>
void writeScalar(std::vector<std::uint8_t> & data, std::size_t offset, const T & value)
{
  std::memcpy(data.data() + offset, &value, sizeof(T));
}

sensor_msgs::msg::PointField makeField(
  const std::string & name, std::uint32_t offset, std::uint8_t datatype)
{
  sensor_msgs::msg::PointField result;
  result.name = name;
  result.offset = offset;
  result.datatype = datatype;
  result.count = 1U;
  return result;
}

}  // namespace

class DualLidarFusionNode : public rclcpp::Node
{
public:
  DualLidarFusionNode()
  : Node("dual_lidar_fusion")
  {
    const std::string mid_topic = declare_parameter<std::string>("mid_topic", "/livox/lidar");
    const std::string hesai_topic = declare_parameter<std::string>("hesai_topic", "/points_raw");
    const std::string output_topic =
      declare_parameter<std::string>("output_topic", "/livox/lidar_fused");
    const std::string diagnostics_topic = declare_parameter<std::string>(
      "diagnostics_topic", "/fastlio_go2w_fusion/diagnostics");
    const std::string debug_topic =
      declare_parameter<std::string>("debug_topic", "/livox/lidar_fused_debug");
    const auto mid_stride = declare_parameter<std::int64_t>("mid_point_stride", 3);
    const auto hesai_stride = declare_parameter<std::int64_t>("hesai_firing_stride", 3);
    const auto max_pending = declare_parameter<std::int64_t>("max_pending_mid_frames", 32);
    const auto input_depth = declare_parameter<std::int64_t>("input_queue_depth", 64);
    const auto output_depth = declare_parameter<std::int64_t>("output_queue_depth", 10);
    const double min_range_m = declare_parameter<double>("min_range_m", 0.5);
    const double time_offset_sec = declare_parameter<double>("hesai_time_offset_sec", 0.0);
    const double max_point_delta_sec =
      declare_parameter<double>("max_hesai_point_header_delta_sec", 1.0);
    const double pending_flush_wall_timeout_sec =
      declare_parameter<double>("pending_flush_wall_timeout_sec", 0.5);
    partial_hesai_min_points_ =
      declare_parameter<std::int64_t>("partial_hesai_min_points", 0);
    partial_hesai_min_span_ns_ = secondsToUnsignedNanoseconds(
      declare_parameter<double>("partial_hesai_min_span_sec", 0.08));
    publish_debug_cloud_ = declare_parameter<bool>("publish_debug_cloud", false);
    const auto translation = declare_parameter<std::vector<double>>(
      "hesai_to_livox.translation", {-0.018602675, 0.0, -0.095450199});
    const auto rotation = declare_parameter<std::vector<double>>(
      "hesai_to_livox.rotation_xyzw",
      {-0.112310121, -0.112310121, 0.698130673, 0.698130673});

    checkPositive("mid_point_stride", mid_stride);
    checkPositive("hesai_firing_stride", hesai_stride);
    checkPositive("max_pending_mid_frames", max_pending);
    checkPositive("input_queue_depth", input_depth);
    checkPositive("output_queue_depth", output_depth);
    if (partial_hesai_min_points_ < 0) {
      throw std::invalid_argument("partial_hesai_min_points must be non-negative");
    }
    if (!std::isfinite(pending_flush_wall_timeout_sec) ||
      pending_flush_wall_timeout_sec < 0.0)
    {
      throw std::invalid_argument("pending_flush_wall_timeout_sec must be finite and non-negative");
    }
    if (translation.size() != 3U || rotation.size() != 4U) {
      throw std::invalid_argument("extrinsic translation/rotation must have 3/4 elements");
    }

    FusionOptions options;
    options.mid_point_stride = static_cast<std::size_t>(mid_stride);
    options.hesai_firing_stride = static_cast<std::size_t>(hesai_stride);
    options.max_pending_mid_frames = static_cast<std::size_t>(max_pending);
    options.min_range_m = min_range_m;
    options.hesai_to_livox.translation = Vec3{translation[0], translation[1], translation[2]};
    options.hesai_to_livox.rotation =
      Quaternion{rotation[0], rotation[1], rotation[2], rotation[3]};
    core_.reset(new FusionCore(options));
    parse_options_.time_offset_ns = secondsToSignedNanoseconds(time_offset_sec);
    parse_options_.max_point_header_delta_ns =
      secondsToUnsignedNanoseconds(max_point_delta_sec);

    const auto input_qos =
      rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(input_depth))).reliable();
    const auto output_qos =
      rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(output_depth))).reliable();
    fused_publisher_ = create_publisher<LivoxMessage>(output_topic, output_qos);
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic, output_qos);
    if (publish_debug_cloud_) {
      debug_publisher_ = create_publisher<PointCloud2>(debug_topic, output_qos);
    }
    mid_subscription_ = create_subscription<LivoxMessage>(
      mid_topic, input_qos,
      std::bind(&DualLidarFusionNode::onMidCloud, this, std::placeholders::_1));
    hesai_subscription_ = create_subscription<PointCloud2>(
      hesai_topic, input_qos,
      std::bind(&DualLidarFusionNode::onHesaiCloud, this, std::placeholders::_1));
    if (pending_flush_wall_timeout_sec > 0.0) {
      idle_flush_timeout_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(pending_flush_wall_timeout_sec));
      idle_flush_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&DualLidarFusionNode::onIdleFlushTimer, this));
    }

    RCLCPP_INFO(
      get_logger(), "Offline fusion ready: %s + %s -> %s", mid_topic.c_str(),
      hesai_topic.c_str(), output_topic.c_str());
  }

private:
  struct Counters
  {
    std::uint64_t mid_messages_received{0U};
    std::uint64_t mid_nonmonotonic_drops{0U};
    std::uint64_t mid_point_count_mismatches{0U};
    std::uint64_t hesai_messages_received{0U};
    std::uint64_t hesai_messages_accepted{0U};
    std::uint64_t hesai_nonmonotonic_drops{0U};
    std::uint64_t hesai_parser_errors{0U};
    std::uint64_t hesai_partial_clouds{0U};
    std::uint64_t hesai_invalid_points{0U};
    std::uint64_t fused_messages_published{0U};
    std::uint64_t mid_only_fallbacks{0U};
    std::uint64_t pending_queue_overflows{0U};
    std::uint64_t idle_flush_frames{0U};
    std::uint64_t mid_points_output{0U};
    std::uint64_t hesai_points_output{0U};
    std::uint64_t hesai_points_stale{0U};
    std::uint64_t hesai_points_filtered{0U};
  } counters_;

  static void checkPositive(const std::string & name, std::int64_t value)
  {
    if (value <= 0) {
      throw std::invalid_argument(name + " must be positive");
    }
  }

  void onMidCloud(const LivoxMessage::SharedPtr message)
  {
    const auto started = std::chrono::steady_clock::now();
    last_input_wall_time_ = started;
    ++counters_.mid_messages_received;
    if (message->timebase == 0U) {
      ++counters_.mid_nonmonotonic_drops;
      finishDiagnostic(started, DiagnosticStatus::ERROR, "MID message has zero timebase");
      return;
    }
    if (message->point_num != message->points.size()) {
      ++counters_.mid_point_count_mismatches;
    }
    MidFrame frame;
    frame.header_stamp_ns = stampToNanoseconds(message->header.stamp);
    frame.timebase_ns = message->timebase;
    frame.frame_id = message->header.frame_id;
    frame.lidar_id = message->lidar_id;
    std::copy(message->rsvd.begin(), message->rsvd.end(), frame.reserved.begin());
    frame.points.reserve(message->points.size());
    for (const auto & point : message->points) {
      frame.max_offset_ns = std::max(frame.max_offset_ns, point.offset_time);
      frame.points.push_back(
        FusedPoint{
          point.offset_time, point.x, point.y, point.z, point.reflectivity,
          point.tag, point.line, PointSource::kMid360});
    }
    if (!core_->enqueueMid(std::move(frame))) {
      ++counters_.mid_nonmonotonic_drops;
      finishDiagnostic(started, DiagnosticStatus::WARN, "dropped non-increasing MID timebase");
      return;
    }
    publishResults(core_->drainReady(), started);
  }

  void onHesaiCloud(const PointCloud2::SharedPtr message)
  {
    const auto started = std::chrono::steady_clock::now();
    last_input_wall_time_ = started;
    ++counters_.hesai_messages_received;
    auto parsed = parseHesaiPointCloud(*message, parse_options_);
    if (!parsed.ok) {
      ++counters_.hesai_parser_errors;
      RCLCPP_WARN(get_logger(), "Rejected Hesai cloud: %s", parsed.error.c_str());
      finishDiagnostic(started, DiagnosticStatus::ERROR, "Hesai parser error: " + parsed.error);
      return;
    }
    const std::uint64_t span_ns = hesaiSpan(parsed.points);
    const bool too_few_points =
      partial_hesai_min_points_ > 0 &&
      static_cast<std::int64_t>(parsed.input_points) < partial_hesai_min_points_;
    const bool short_valid_cloud = too_few_points ||
      span_ns < partial_hesai_min_span_ns_;
    if (parsed.partial || short_valid_cloud) {
      ++counters_.hesai_partial_clouds;
    }
    counters_.hesai_invalid_points += parsed.invalid_points;
    const auto report = core_->pushHesaiCloud(
      parsed.adjusted_header_stamp_ns, std::move(parsed.points));
    if (!report.accepted) {
      if (report.nonmonotonic) {
        ++counters_.hesai_nonmonotonic_drops;
      }
      finishDiagnostic(started, DiagnosticStatus::WARN, "dropped non-increasing Hesai cloud");
      return;
    }
    ++counters_.hesai_messages_accepted;
    publishResults(core_->drainReady(), started);
  }

  void onIdleFlushTimer()
  {
    const auto started = std::chrono::steady_clock::now();
    if (core_->pendingMidCount() == 0U ||
      started - last_input_wall_time_ < idle_flush_timeout_)
    {
      return;
    }
    publishResults(core_->flushPendingMidOnly(), started);
  }

  static std::uint64_t hesaiSpan(const std::vector<HesaiPoint> & points)
  {
    if (points.empty()) {
      return 0U;
    }
    auto range = std::minmax_element(
      points.begin(), points.end(), [](const HesaiPoint & lhs, const HesaiPoint & rhs) {
        return lhs.timestamp_ns < rhs.timestamp_ns;
      });
    return range.second->timestamp_ns - range.first->timestamp_ns;
  }

  void publishResults(
    const std::vector<FusionResult> & results,
    const std::chrono::steady_clock::time_point & started)
  {
    if (results.empty()) {
      finishDiagnostic(started, DiagnosticStatus::OK, "waiting for complete Hesai coverage");
      return;
    }
    for (const auto & result : results) {
      counters_.mid_points_output += result.stats.mid_points_output;
      counters_.hesai_points_output += result.stats.hesai_points_output;
      counters_.hesai_points_stale += result.stats.hesai_points_stale;
      counters_.hesai_points_filtered += result.stats.hesai_points_filtered;
      std::string diagnostic_message = "fused frame published";
      if (result.stats.mid_only_fallback) {
        ++counters_.mid_only_fallbacks;
        if (result.stats.mid_only_reason == MidOnlyReason::kFlush) {
          ++counters_.idle_flush_frames;
          diagnostic_message = "MID-only idle flush";
        } else if (result.stats.mid_only_reason == MidOnlyReason::kQueueOverflow) {
          ++counters_.pending_queue_overflows;
          diagnostic_message = "MID-only queue fallback";
        } else {
          diagnostic_message = "MID-only fallback";
        }
      }
      const auto output = makeLivoxMessage(result);
      fused_publisher_->publish(output);
      if (publish_debug_cloud_) {
        debug_publisher_->publish(makeDebugCloud(result));
      }
      ++counters_.fused_messages_published;
      finishDiagnostic(
        started, result.stats.mid_only_fallback ? DiagnosticStatus::WARN : DiagnosticStatus::OK,
        diagnostic_message);
    }
  }

  static LivoxMessage makeLivoxMessage(const FusionResult & result)
  {
    LivoxMessage output;
    output.header.frame_id = result.frame.frame_id;
    output.header.stamp = nanosecondsToStamp(
      result.frame.header_stamp_ns == 0U ? result.frame.timebase_ns :
      result.frame.header_stamp_ns);
    output.timebase = result.frame.timebase_ns;
    output.lidar_id = result.frame.lidar_id;
    std::copy(result.frame.reserved.begin(), result.frame.reserved.end(), output.rsvd.begin());
    if (result.frame.points.size() > std::numeric_limits<std::uint32_t>::max()) {
      throw std::overflow_error("fused point count exceeds CustomMsg capacity");
    }
    output.point_num = static_cast<std::uint32_t>(result.frame.points.size());
    output.points.reserve(result.frame.points.size());
    for (const auto & point : result.frame.points) {
      livox_ros_driver2::msg::CustomPoint converted;
      converted.offset_time = point.offset_time;
      converted.x = point.x;
      converted.y = point.y;
      converted.z = point.z;
      converted.reflectivity = point.reflectivity;
      converted.tag = point.tag;
      converted.line = point.line;
      output.points.push_back(converted);
    }
    return output;
  }

  static PointCloud2 makeDebugCloud(const FusionResult & result)
  {
    PointCloud2 cloud;
    cloud.header.frame_id = result.frame.frame_id;
    cloud.header.stamp = nanosecondsToStamp(
      result.frame.header_stamp_ns == 0U ? result.frame.timebase_ns :
      result.frame.header_stamp_ns);
    cloud.height = 1U;
    cloud.width = static_cast<std::uint32_t>(result.frame.points.size());
    cloud.fields = {
      makeField("x", 0U, sensor_msgs::msg::PointField::FLOAT32),
      makeField("y", 4U, sensor_msgs::msg::PointField::FLOAT32),
      makeField("z", 8U, sensor_msgs::msg::PointField::FLOAT32),
      makeField("intensity", 12U, sensor_msgs::msg::PointField::FLOAT32),
      makeField("offset_time", 16U, sensor_msgs::msg::PointField::UINT32),
      makeField("line", 20U, sensor_msgs::msg::PointField::UINT8),
      makeField("source", 21U, sensor_msgs::msg::PointField::UINT8)};
    cloud.point_step = 24U;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.data.resize(cloud.row_step);
    for (std::size_t index = 0U; index < result.frame.points.size(); ++index) {
      const auto & point = result.frame.points[index];
      const std::size_t base = index * cloud.point_step;
      writeScalar(cloud.data, base + 0U, point.x);
      writeScalar(cloud.data, base + 4U, point.y);
      writeScalar(cloud.data, base + 8U, point.z);
      const float intensity = point.reflectivity;
      writeScalar(cloud.data, base + 12U, intensity);
      writeScalar(cloud.data, base + 16U, point.offset_time);
      cloud.data[base + 20U] = point.line;
      cloud.data[base + 21U] = static_cast<std::uint8_t>(point.source);
    }
    return cloud;
  }

  void finishDiagnostic(
    const std::chrono::steady_clock::time_point & started, std::uint8_t level,
    const std::string & message)
  {
    last_processing_time_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    DiagnosticStatus status;
    status.level = level;
    status.name = get_fully_qualified_name() + std::string(": fusion");
    status.hardware_id = "offline_mid360_pandar_xt16";
    status.message = message;
    addValue(status, "mid_messages_received", counters_.mid_messages_received);
    addValue(status, "mid_nonmonotonic_drops", counters_.mid_nonmonotonic_drops);
    addValue(status, "mid_point_count_mismatches", counters_.mid_point_count_mismatches);
    addValue(status, "hesai_messages_received", counters_.hesai_messages_received);
    addValue(status, "hesai_messages_accepted", counters_.hesai_messages_accepted);
    addValue(status, "hesai_nonmonotonic_drops", counters_.hesai_nonmonotonic_drops);
    addValue(status, "hesai_parser_errors", counters_.hesai_parser_errors);
    addValue(status, "hesai_partial_clouds", counters_.hesai_partial_clouds);
    addValue(status, "hesai_invalid_points", counters_.hesai_invalid_points);
    addValue(status, "fused_messages_published", counters_.fused_messages_published);
    addValue(status, "mid_only_fallbacks", counters_.mid_only_fallbacks);
    addValue(status, "pending_queue_overflows", counters_.pending_queue_overflows);
    addValue(status, "idle_flush_frames", counters_.idle_flush_frames);
    addValue(status, "mid_points_output", counters_.mid_points_output);
    addValue(status, "hesai_points_output", counters_.hesai_points_output);
    addValue(status, "hesai_points_stale", counters_.hesai_points_stale);
    addValue(status, "hesai_points_filtered", counters_.hesai_points_filtered);
    addValue(status, "pending_mid_frames", core_->pendingMidCount());
    addValue(status, "buffered_hesai_points", core_->bufferedHesaiPointCount());
    diagnostic_msgs::msg::KeyValue processing;
    processing.key = "last_processing_time_ms";
    processing.value = std::to_string(last_processing_time_ms_);
    status.values.push_back(processing);
    array.status.push_back(status);
    diagnostics_publisher_->publish(array);
  }

  template<typename T>
  static void addValue(DiagnosticStatus & status, const std::string & key, T value)
  {
    diagnostic_msgs::msg::KeyValue entry;
    entry.key = key;
    entry.value = std::to_string(value);
    status.values.push_back(entry);
  }

  HesaiParseOptions parse_options_;
  std::unique_ptr<FusionCore> core_;
  std::chrono::steady_clock::time_point last_input_wall_time_{
    std::chrono::steady_clock::now()};
  std::chrono::steady_clock::duration idle_flush_timeout_{
    std::chrono::steady_clock::duration::zero()};
  bool publish_debug_cloud_{false};
  std::int64_t partial_hesai_min_points_{50000};
  std::uint64_t partial_hesai_min_span_ns_{80000000U};
  double last_processing_time_ms_{0.0};
  rclcpp::Publisher<LivoxMessage>::SharedPtr fused_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_publisher_;
  rclcpp::Publisher<PointCloud2>::SharedPtr debug_publisher_;
  rclcpp::Subscription<LivoxMessage>::SharedPtr mid_subscription_;
  rclcpp::Subscription<PointCloud2>::SharedPtr hesai_subscription_;
  rclcpp::TimerBase::SharedPtr idle_flush_timer_;
};

}  // namespace fastlio_go2w_fusion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<fastlio_go2w_fusion::DualLidarFusionNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("dual_lidar_fusion"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
