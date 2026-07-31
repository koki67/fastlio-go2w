#include "fastlio_go2w_livox/pointcloud_adapter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <sensor_msgs/msg/point_field.hpp>

namespace fastlio_go2w_livox
{
namespace
{

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;

constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;

bool host_is_big_endian()
{
  const std::uint16_t marker = 0x0102;
  return *reinterpret_cast<const std::uint8_t *>(&marker) == 0x01;
}

template<typename T>
T read_scalar(const std::uint8_t * source, bool source_is_big_endian)
{
  std::array<std::uint8_t, sizeof(T)> bytes{};
  std::memcpy(bytes.data(), source, sizeof(T));
  if (source_is_big_endian != host_is_big_endian()) {
    std::reverse(bytes.begin(), bytes.end());
  }
  T value{};
  std::memcpy(&value, bytes.data(), sizeof(T));
  return value;
}

std::optional<std::size_t> datatype_size(std::uint8_t datatype)
{
  switch (datatype) {
    case PointField::INT8:
    case PointField::UINT8:
      return 1;
    case PointField::INT16:
    case PointField::UINT16:
      return 2;
    case PointField::INT32:
    case PointField::UINT32:
    case PointField::FLOAT32:
      return 4;
    case PointField::FLOAT64:
      return 8;
    default:
      return std::nullopt;
  }
}

bool stamp_to_nanoseconds(
  const builtin_interfaces::msg::Time & stamp, std::int64_t & nanoseconds)
{
  if (stamp.sec < 0 || stamp.nanosec >= static_cast<std::uint32_t>(kNanosecondsPerSecond)) {
    return false;
  }
  const auto seconds = static_cast<std::int64_t>(stamp.sec);
  if (seconds > (std::numeric_limits<std::int64_t>::max() - stamp.nanosec) /
    kNanosecondsPerSecond)
  {
    return false;
  }
  nanoseconds = seconds * kNanosecondsPerSecond + stamp.nanosec;
  return nanoseconds > 0;
}

bool round_timestamp_nanoseconds(double value, std::int64_t & result)
{
  if (!std::isfinite(value)) {
    return false;
  }
  const auto wide = static_cast<long double>(value);
  if (wide < static_cast<long double>(std::numeric_limits<std::int64_t>::min()) ||
    wide > static_cast<long double>(std::numeric_limits<std::int64_t>::max()))
  {
    return false;
  }
  result = static_cast<std::int64_t>(std::llround(value));
  return true;
}

std::uint64_t half_ulp_nanoseconds(double value)
{
  const double next = std::nextafter(value, std::numeric_limits<double>::infinity());
  const double ulp = next - value;
  if (!std::isfinite(ulp) || ulp < 2.0) {
    return 0;
  }
  return static_cast<std::uint64_t>(std::floor(ulp * 0.5));
}

struct SourceFields
{
  const PointField * x{nullptr};
  const PointField * y{nullptr};
  const PointField * z{nullptr};
  const PointField * intensity{nullptr};
  const PointField * tag{nullptr};
  const PointField * line{nullptr};
  const PointField * timestamp{nullptr};
};

bool find_source_fields(const PointCloud2 & input, SourceFields & result)
{
  std::unordered_map<std::string, const PointField *> required;
  for (const auto & field : input.fields) {
    if (field.name != "x" && field.name != "y" && field.name != "z" &&
      field.name != "intensity" && field.name != "tag" && field.name != "line" &&
      field.name != "timestamp")
    {
      continue;
    }
    if (!required.emplace(field.name, &field).second) {
      return false;
    }
  }
  if (required.size() != 7) {
    return false;
  }

  result.x = required.at("x");
  result.y = required.at("y");
  result.z = required.at("z");
  result.intensity = required.at("intensity");
  result.tag = required.at("tag");
  result.line = required.at("line");
  result.timestamp = required.at("timestamp");

  for (const auto * field : {result.x, result.y, result.z, result.intensity}) {
    if (field->datatype != PointField::FLOAT32 || field->count != 1) {
      return false;
    }
  }
  for (const auto * field : {result.tag, result.line}) {
    if (field->datatype != PointField::UINT8 || field->count != 1) {
      return false;
    }
  }
  if (result.timestamp->datatype != PointField::FLOAT64 || result.timestamp->count != 1) {
    return false;
  }
  for (const auto * field :
    {result.x, result.y, result.z, result.intensity, result.tag, result.line, result.timestamp})
  {
    const auto size = datatype_size(field->datatype);
    if (!size || field->offset > input.point_step || *size > input.point_step - field->offset) {
      return false;
    }
  }
  std::vector<std::pair<std::uint32_t, std::uint32_t>> ranges;
  for (const auto * field :
    {result.x, result.y, result.z, result.intensity, result.tag, result.line, result.timestamp})
  {
    const auto size = static_cast<std::uint32_t>(*datatype_size(field->datatype));
    ranges.emplace_back(field->offset, field->offset + size);
  }
  std::sort(ranges.begin(), ranges.end());
  for (std::size_t index = 1; index < ranges.size(); ++index) {
    if (ranges[index].first < ranges[index - 1].second) {
      return false;
    }
  }
  return true;
}

bool valid_layout(const PointCloud2 & input, std::uint64_t & total_points)
{
  if (input.height == 0 || input.width == 0 || input.point_step == 0) {
    return false;
  }
  total_points = static_cast<std::uint64_t>(input.width) * input.height;
  if (total_points > std::numeric_limits<std::uint32_t>::max() ||
    total_points > std::numeric_limits<std::size_t>::max())
  {
    return false;
  }
  const std::uint64_t packed_row =
    static_cast<std::uint64_t>(input.width) * input.point_step;
  if (packed_row > std::numeric_limits<std::uint32_t>::max() || input.row_step < packed_row) {
    return false;
  }
  const std::uint64_t required_data =
    static_cast<std::uint64_t>(input.row_step) * input.height;
  return required_data <= input.data.size() && required_data <= std::numeric_limits<std::size_t>::max();
}

struct ParsedPoint
{
  livox_ros_driver2::msg::CustomPoint point;
  std::int64_t timestamp_ns;
};

}  // namespace

std::uint64_t AdapterStats::critical_drops() const noexcept
{
  return dropped_invalid_header + dropped_header_regression + dropped_schema +
         dropped_layout + dropped_nonfinite_coordinate + dropped_nonfinite_intensity +
         dropped_nonfinite_timestamp + dropped_intensity_out_of_range +
         dropped_intensity_nonintegral + dropped_negative_offset +
         dropped_offset_too_large + dropped_too_few_points;
}

PointcloudAdapter::PointcloudAdapter(AdapterConfig config)
: config_(std::move(config))
{
  if (!std::isfinite(config_.max_point_header_delta_sec) ||
    config_.max_point_header_delta_sec <= 0.0)
  {
    throw std::invalid_argument("max_point_header_delta_sec must be finite and positive");
  }
  const double maximum_encodable_delta_sec =
    static_cast<double>(std::numeric_limits<std::uint32_t>::max()) /
    static_cast<double>(kNanosecondsPerSecond);
  if (config_.max_point_header_delta_sec > maximum_encodable_delta_sec) {
    throw std::invalid_argument("max_point_header_delta_sec exceeds the uint32 offset range");
  }
  if (config_.minimum_points == 0) {
    throw std::invalid_argument("minimum_points must be positive");
  }
}

ConversionResult PointcloudAdapter::convert(const PointCloud2 & input)
{
  ++stats_.received_frames;
  ConversionResult result;

  std::int64_t header_stamp_ns = 0;
  if (!stamp_to_nanoseconds(input.header.stamp, header_stamp_ns)) {
    ++stats_.dropped_invalid_header;
    result.drop_reason = "invalid_header";
    return result;
  }
  if (last_header_stamp_ns_ && header_stamp_ns < *last_header_stamp_ns_) {
    ++stats_.dropped_header_regression;
    result.drop_reason = "header_regression";
    return result;
  }
  last_header_stamp_ns_ = header_stamp_ns;

  SourceFields fields;
  if (!find_source_fields(input, fields)) {
    ++stats_.dropped_schema;
    result.drop_reason = "schema";
    return result;
  }

  std::uint64_t total_points = 0;
  if (!valid_layout(input, total_points)) {
    ++stats_.dropped_layout;
    result.drop_reason = "layout";
    return result;
  }
  if (total_points < config_.minimum_points) {
    ++stats_.dropped_too_few_points;
    result.drop_reason = "too_few_points";
    return result;
  }

  const auto maximum_delta_ns_wide =
    static_cast<long double>(config_.max_point_header_delta_sec) * kNanosecondsPerSecond;
  const auto maximum_delta_ns = static_cast<std::uint64_t>(std::llround(maximum_delta_ns_wide));
  std::vector<ParsedPoint> points;
  points.reserve(static_cast<std::size_t>(total_points));
  std::int64_t minimum_timestamp_ns = std::numeric_limits<std::int64_t>::max();
  std::int64_t maximum_timestamp_ns = std::numeric_limits<std::int64_t>::min();
  bool reordered = false;
  bool quantization_clamped = false;
  std::uint64_t quantization_clamped_points = 0;
  std::optional<std::int64_t> previous_timestamp_ns;

  for (std::uint32_t row = 0; row < input.height; ++row) {
    for (std::uint32_t column = 0; column < input.width; ++column) {
      const auto point_offset =
        static_cast<std::size_t>(row) * input.row_step +
        static_cast<std::size_t>(column) * input.point_step;
      const auto * base = input.data.data() + point_offset;

      const auto x = read_scalar<float>(base + fields.x->offset, input.is_bigendian);
      const auto y = read_scalar<float>(base + fields.y->offset, input.is_bigendian);
      const auto z = read_scalar<float>(base + fields.z->offset, input.is_bigendian);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        ++stats_.dropped_nonfinite_coordinate;
        result.drop_reason = "nonfinite_coordinate";
        return result;
      }

      const auto intensity =
        read_scalar<float>(base + fields.intensity->offset, input.is_bigendian);
      if (!std::isfinite(intensity)) {
        ++stats_.dropped_nonfinite_intensity;
        result.drop_reason = "nonfinite_intensity";
        return result;
      }
      if (intensity < 0.0F || intensity > 255.0F) {
        ++stats_.dropped_intensity_out_of_range;
        result.drop_reason = "intensity_out_of_range";
        return result;
      }
      if (std::trunc(intensity) != intensity) {
        ++stats_.dropped_intensity_nonintegral;
        result.drop_reason = "intensity_nonintegral";
        return result;
      }

      const auto timestamp =
        read_scalar<double>(base + fields.timestamp->offset, input.is_bigendian);
      if (!std::isfinite(timestamp)) {
        ++stats_.dropped_nonfinite_timestamp;
        result.drop_reason = "nonfinite_timestamp";
        return result;
      }
      std::int64_t timestamp_ns = 0;
      if (!round_timestamp_nanoseconds(timestamp, timestamp_ns)) {
        ++stats_.dropped_offset_too_large;
        result.drop_reason = "offset_too_large";
        return result;
      }
      if (timestamp_ns < header_stamp_ns) {
        const auto negative_delta_ns =
          static_cast<std::uint64_t>(header_stamp_ns - timestamp_ns);
        if (negative_delta_ns <= half_ulp_nanoseconds(timestamp)) {
          timestamp_ns = header_stamp_ns;
          quantization_clamped = true;
          ++quantization_clamped_points;
        } else {
          ++stats_.dropped_negative_offset;
          result.drop_reason = "negative_offset";
          return result;
        }
      }
      const auto delta_ns = static_cast<std::uint64_t>(timestamp_ns - header_stamp_ns);
      if (delta_ns > std::numeric_limits<std::uint32_t>::max() ||
        delta_ns > maximum_delta_ns)
      {
        ++stats_.dropped_offset_too_large;
        result.drop_reason = "offset_too_large";
        return result;
      }

      if (previous_timestamp_ns && timestamp_ns < *previous_timestamp_ns) {
        reordered = true;
      }
      previous_timestamp_ns = timestamp_ns;
      minimum_timestamp_ns = std::min(minimum_timestamp_ns, timestamp_ns);
      maximum_timestamp_ns = std::max(maximum_timestamp_ns, timestamp_ns);

      livox_ros_driver2::msg::CustomPoint point;
      point.offset_time = static_cast<std::uint32_t>(delta_ns);
      point.x = x;
      point.y = y;
      point.z = z;
      point.reflectivity = static_cast<std::uint8_t>(intensity);
      point.tag = read_scalar<std::uint8_t>(base + fields.tag->offset, input.is_bigendian);
      point.line = read_scalar<std::uint8_t>(base + fields.line->offset, input.is_bigendian);
      points.push_back({point, timestamp_ns});
    }
  }

  if (reordered) {
    std::stable_sort(
      points.begin(), points.end(),
      [](const ParsedPoint & left, const ParsedPoint & right) {
        return left.timestamp_ns < right.timestamp_ns;
      });
    ++stats_.reordered_frames;
  }
  if (quantization_clamped) {
    ++stats_.quantization_clamped_frames;
    stats_.quantization_clamped_points += quantization_clamped_points;
  }

  result.message.header = input.header;
  result.message.timebase = static_cast<std::uint64_t>(header_stamp_ns);
  result.message.point_num = static_cast<std::uint32_t>(points.size());
  result.message.lidar_id = 0;
  result.message.rsvd.fill(0);
  result.message.points.reserve(points.size());
  for (const auto & parsed : points) {
    result.message.points.push_back(parsed.point);
  }

  result.converted = true;
  result.reordered = reordered;
  result.scan_width_sec =
    static_cast<double>(maximum_timestamp_ns - minimum_timestamp_ns) / kNanosecondsPerSecond;
  ++stats_.converted_frames;
  stats_.latest_scan_width_sec = result.scan_width_sec;
  return result;
}

}  // namespace fastlio_go2w_livox
