#include "fastlio_go2w_hesai/pointcloud_adapter.hpp"

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

namespace fastlio_go2w_hesai
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

template<typename T>
void write_little_endian(std::uint8_t * destination, T value)
{
  std::array<std::uint8_t, sizeof(T)> bytes{};
  std::memcpy(bytes.data(), &value, sizeof(T));
  if (host_is_big_endian()) {
    std::reverse(bytes.begin(), bytes.end());
  }
  std::memcpy(destination, bytes.data(), sizeof(T));
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
  return true;
}

bool shifted_stamp(
  std::int64_t source_ns, double offset_sec, builtin_interfaces::msg::Time & output)
{
  if (!std::isfinite(offset_sec)) {
    return false;
  }
  const long double offset_ns_value =
    static_cast<long double>(offset_sec) * static_cast<long double>(kNanosecondsPerSecond);
  if (offset_ns_value < static_cast<long double>(std::numeric_limits<std::int64_t>::min()) ||
    offset_ns_value > static_cast<long double>(std::numeric_limits<std::int64_t>::max()))
  {
    return false;
  }
  const auto offset_ns = static_cast<std::int64_t>(std::llround(offset_ns_value));
  if ((offset_ns > 0 && source_ns > std::numeric_limits<std::int64_t>::max() - offset_ns) ||
    (offset_ns < 0 && source_ns < std::numeric_limits<std::int64_t>::min() - offset_ns))
  {
    return false;
  }
  const auto result_ns = source_ns + offset_ns;
  if (result_ns < 0) {
    return false;
  }
  const auto seconds = result_ns / kNanosecondsPerSecond;
  if (seconds > std::numeric_limits<std::int32_t>::max()) {
    return false;
  }
  output.sec = static_cast<std::int32_t>(seconds);
  output.nanosec = static_cast<std::uint32_t>(result_ns % kNanosecondsPerSecond);
  return true;
}

PointField output_field(const std::string & name, std::uint32_t offset, std::uint8_t datatype)
{
  PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  return field;
}

struct SourceFields
{
  const PointField * x{nullptr};
  const PointField * y{nullptr};
  const PointField * z{nullptr};
  const PointField * intensity{nullptr};
  const PointField * timestamp{nullptr};
  const PointField * ring{nullptr};
};

bool find_source_fields(const PointCloud2 & input, SourceFields & result)
{
  std::unordered_map<std::string, const PointField *> required;
  for (const auto & field : input.fields) {
    if (field.name != "x" && field.name != "y" && field.name != "z" &&
      field.name != "intensity" && field.name != "timestamp" && field.name != "ring")
    {
      continue;
    }
    if (!required.emplace(field.name, &field).second) {
      return false;
    }
  }
  if (required.size() != 6) {
    return false;
  }
  result.x = required.at("x");
  result.y = required.at("y");
  result.z = required.at("z");
  result.intensity = required.at("intensity");
  result.timestamp = required.at("timestamp");
  result.ring = required.at("ring");

  const std::array<const PointField *, 4> float_fields{
    result.x, result.y, result.z, result.intensity};
  for (const auto * field : float_fields) {
    if (field->datatype != PointField::FLOAT32 || field->count != 1) {
      return false;
    }
  }
  if (result.timestamp->datatype != PointField::FLOAT64 || result.timestamp->count != 1 ||
    result.ring->datatype != PointField::UINT16 || result.ring->count != 1)
  {
    return false;
  }
  for (const auto * field :
    {result.x, result.y, result.z, result.intensity, result.timestamp, result.ring})
  {
    const auto size = datatype_size(field->datatype);
    if (!size || field->offset > input.point_step || *size > input.point_step - field->offset) {
      return false;
    }
  }
  return true;
}

bool valid_layout(const PointCloud2 & input)
{
  if (input.height == 0 || input.point_step == 0) {
    return false;
  }
  const std::uint64_t packed_row =
    static_cast<std::uint64_t>(input.width) * input.point_step;
  if (packed_row > std::numeric_limits<std::uint32_t>::max() || input.row_step < packed_row) {
    return false;
  }
  const std::uint64_t required_data =
    static_cast<std::uint64_t>(input.row_step) * input.height;
  return required_data <= input.data.size();
}

struct Point
{
  float x;
  float y;
  float z;
  float intensity;
  double timestamp;
  std::uint16_t ring;
};

}  // namespace

PointcloudAdapter::PointcloudAdapter(AdapterConfig config)
: config_(std::move(config))
{
  if (!std::isfinite(config_.lidar_time_offset_sec)) {
    throw std::invalid_argument("lidar_time_offset_sec must be finite");
  }
  if (!std::isfinite(config_.max_point_header_delta_sec) ||
    config_.max_point_header_delta_sec <= 0.0)
  {
    throw std::invalid_argument("max_point_header_delta_sec must be finite and positive");
  }
  if (config_.ring_count == 0) {
    throw std::invalid_argument("ring_count must be positive");
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

  SourceFields fields;
  if (!find_source_fields(input, fields)) {
    ++stats_.dropped_schema;
    result.drop_reason = "schema";
    return result;
  }
  if (!valid_layout(input)) {
    ++stats_.dropped_layout;
    result.drop_reason = "layout";
    return result;
  }

  const double header_stamp_sec = static_cast<double>(header_stamp_ns) / kNanosecondsPerSecond;
  const auto total_points = static_cast<std::uint64_t>(input.width) * input.height;
  std::vector<Point> points;
  points.reserve(static_cast<std::size_t>(total_points));
  double minimum_timestamp = std::numeric_limits<double>::infinity();
  double maximum_timestamp = -std::numeric_limits<double>::infinity();

  for (std::uint32_t row = 0; row < input.height; ++row) {
    for (std::uint32_t column = 0; column < input.width; ++column) {
      const auto point_offset =
        static_cast<std::size_t>(row) * input.row_step +
        static_cast<std::size_t>(column) * input.point_step;
      const auto * base = input.data.data() + point_offset;
      const double timestamp =
        read_scalar<double>(base + fields.timestamp->offset, input.is_bigendian);
      if (!std::isfinite(timestamp)) {
        ++stats_.dropped_nonfinite_timestamp;
        result.drop_reason = "nonfinite_timestamp";
        return result;
      }
      const double relative_time = timestamp - header_stamp_sec;
      if (!std::isfinite(relative_time) ||
        std::abs(relative_time) > config_.max_point_header_delta_sec)
      {
        ++stats_.dropped_timestamp_out_of_range;
        result.drop_reason = "timestamp_out_of_range";
        return result;
      }

      Point point{
        read_scalar<float>(base + fields.x->offset, input.is_bigendian),
        read_scalar<float>(base + fields.y->offset, input.is_bigendian),
        read_scalar<float>(base + fields.z->offset, input.is_bigendian),
        read_scalar<float>(base + fields.intensity->offset, input.is_bigendian),
        timestamp,
        read_scalar<std::uint16_t>(base + fields.ring->offset, input.is_bigendian)};
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || !std::isfinite(point.intensity) ||
        point.ring >= config_.ring_count)
      {
        ++result.invalid_points;
        ++stats_.invalid_points;
        continue;
      }
      minimum_timestamp = std::min(minimum_timestamp, timestamp);
      maximum_timestamp = std::max(maximum_timestamp, timestamp);
      points.push_back(point);
    }
  }

  if (points.size() < config_.minimum_points) {
    ++stats_.dropped_too_few_points;
    result.drop_reason = "too_few_points";
    return result;
  }

  std::stable_sort(
    points.begin(), points.end(),
    [](const Point & left, const Point & right) {return left.timestamp < right.timestamp;});
  result.scan_width_sec = maximum_timestamp - minimum_timestamp;
  stats_.latest_scan_width_sec = result.scan_width_sec;

  result.cloud.header = input.header;
  if (!shifted_stamp(
      header_stamp_ns, config_.lidar_time_offset_sec, result.cloud.header.stamp))
  {
    ++stats_.dropped_invalid_header;
    result.drop_reason = "invalid_shifted_header";
    return result;
  }
  result.cloud.height = 1;
  result.cloud.width = static_cast<std::uint32_t>(points.size());
  result.cloud.fields = {
    output_field("x", 0, PointField::FLOAT32),
    output_field("y", 4, PointField::FLOAT32),
    output_field("z", 8, PointField::FLOAT32),
    output_field("intensity", 12, PointField::FLOAT32),
    output_field("time", 16, PointField::FLOAT32),
    output_field("ring", 20, PointField::UINT16)};
  result.cloud.is_bigendian = false;
  result.cloud.point_step = 22;
  result.cloud.row_step = result.cloud.width * result.cloud.point_step;
  result.cloud.data.resize(result.cloud.row_step);
  result.cloud.is_dense = true;

  for (std::size_t index = 0; index < points.size(); ++index) {
    auto * destination = result.cloud.data.data() + index * result.cloud.point_step;
    const auto & point = points[index];
    const float relative_time = static_cast<float>(point.timestamp - header_stamp_sec);
    write_little_endian(destination + 0, point.x);
    write_little_endian(destination + 4, point.y);
    write_little_endian(destination + 8, point.z);
    write_little_endian(destination + 12, point.intensity);
    write_little_endian(destination + 16, relative_time);
    write_little_endian(destination + 20, point.ring);
  }

  result.converted = true;
  last_header_stamp_ns_ = header_stamp_ns;
  ++stats_.converted_frames;
  return result;
}

}  // namespace fastlio_go2w_hesai
