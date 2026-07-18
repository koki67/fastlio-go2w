// Copyright 2026 Koki Tanaka
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "fastlio_go2w_fusion/pointcloud_parser.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>
#include <type_traits>

#include "sensor_msgs/msg/point_field.hpp"

namespace fastlio_go2w_fusion
{
namespace
{

using PointField = sensor_msgs::msg::PointField;

bool hostIsBigEndian()
{
  const std::uint16_t value = 0x0102U;
  const auto * bytes = reinterpret_cast<const std::uint8_t *>(&value);
  return bytes[0] == 0x01U;
}

template<typename T>
T readScalar(const std::uint8_t * data, bool data_is_bigendian)
{
  static_assert(std::is_trivially_copyable<T>::value, "scalar must be copyable");
  T value{};
  if (data_is_bigendian == hostIsBigEndian()) {
    std::memcpy(&value, data, sizeof(T));
    return value;
  }
  std::uint8_t reversed[sizeof(T)];
  std::reverse_copy(data, data + sizeof(T), reversed);
  std::memcpy(&value, reversed, sizeof(T));
  return value;
}

const PointField * findField(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name,
  std::string & error)
{
  const PointField * match = nullptr;
  for (const auto & field : cloud.fields) {
    if (field.name != name) {
      continue;
    }
    if (match != nullptr) {
      error = "duplicate PointCloud2 field: " + name;
      return nullptr;
    }
    match = &field;
  }
  if (match == nullptr) {
    error = "missing PointCloud2 field: " + name;
  }
  return match;
}

bool checkField(
  const PointField * field, const std::string & name, std::uint8_t datatype,
  std::size_t size, std::uint32_t point_step, std::string & error)
{
  if (field == nullptr) {
    return false;
  }
  if (field->count != 1U) {
    error = "PointCloud2 field '" + name + "' must be scalar";
    return false;
  }
  if (field->datatype != datatype) {
    std::ostringstream stream;
    stream << "PointCloud2 field '" << name << "' has datatype "
           << static_cast<unsigned int>(field->datatype) << ", expected "
           << static_cast<unsigned int>(datatype);
    error = stream.str();
    return false;
  }
  if (field->offset > point_step || size > point_step - field->offset) {
    error = "PointCloud2 field '" + name + "' extends past point_step";
    return false;
  }
  return true;
}

bool stampToNanoseconds(
  const builtin_interfaces::msg::Time & stamp, std::uint64_t & nanoseconds,
  std::string & error)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000U) {
    error = "PointCloud2 header has an invalid timestamp";
    return false;
  }
  const std::uint64_t seconds = static_cast<std::uint64_t>(stamp.sec);
  if (seconds >
    (std::numeric_limits<std::uint64_t>::max() - stamp.nanosec) / 1000000000ULL)
  {
    error = "PointCloud2 header timestamp overflows uint64 nanoseconds";
    return false;
  }
  nanoseconds = seconds * 1000000000ULL + stamp.nanosec;
  return true;
}

bool secondsToNanoseconds(double seconds, std::uint64_t & nanoseconds)
{
  if (!std::isfinite(seconds) || seconds < 0.0) {
    return false;
  }
  const long double scaled = static_cast<long double>(seconds) * 1000000000.0L;
  if (scaled > static_cast<long double>(std::numeric_limits<std::uint64_t>::max())) {
    return false;
  }
  nanoseconds = static_cast<std::uint64_t>(scaled + 0.5L);
  return true;
}

bool addSignedOffset(
  std::uint64_t timestamp, std::int64_t offset, std::uint64_t & adjusted)
{
  if (offset >= 0) {
    const auto positive = static_cast<std::uint64_t>(offset);
    if (timestamp > std::numeric_limits<std::uint64_t>::max() - positive) {
      return false;
    }
    adjusted = timestamp + positive;
    return true;
  }

  // This expression is also defined for INT64_MIN.
  const std::uint64_t magnitude =
    static_cast<std::uint64_t>(-(offset + 1)) + 1U;
  if (timestamp < magnitude) {
    return false;
  }
  adjusted = timestamp - magnitude;
  return true;
}

std::uint64_t absoluteDifference(std::uint64_t lhs, std::uint64_t rhs)
{
  return lhs >= rhs ? lhs - rhs : rhs - lhs;
}

}  // namespace

HesaiParseResult parseHesaiPointCloud(
  const sensor_msgs::msg::PointCloud2 & cloud, const HesaiParseOptions & options)
{
  HesaiParseResult result;
  std::uint64_t header_stamp_ns = 0U;
  if (!stampToNanoseconds(cloud.header.stamp, header_stamp_ns, result.error)) {
    return result;
  }
  if (!addSignedOffset(
      header_stamp_ns, options.time_offset_ns, result.adjusted_header_stamp_ns))
  {
    result.error = "Hesai time offset makes the cloud header timestamp invalid";
    return result;
  }
  if (cloud.point_step == 0U) {
    result.error = "PointCloud2 point_step must be nonzero";
    return result;
  }
  if (cloud.width != 0U && cloud.point_step >
    std::numeric_limits<std::uint32_t>::max() / cloud.width)
  {
    result.error = "PointCloud2 width * point_step overflows uint32";
    return result;
  }
  const std::uint32_t packed_row_size = cloud.width * cloud.point_step;
  if (cloud.row_step < packed_row_size) {
    result.error = "PointCloud2 row_step is smaller than width * point_step";
    return result;
  }
  if (cloud.height != 0U && cloud.row_step >
    std::numeric_limits<std::size_t>::max() / cloud.height)
  {
    result.error = "PointCloud2 row storage size overflows size_t";
    return result;
  }
  const std::size_t required_bytes =
    static_cast<std::size_t>(cloud.row_step) * cloud.height;
  if (cloud.data.size() < required_bytes) {
    result.error = "PointCloud2 data is shorter than row_step * height";
    return result;
  }

  std::string field_error;
  const PointField * x = findField(cloud, "x", field_error);
  if (!field_error.empty()) {result.error = field_error; return result;}
  const PointField * y = findField(cloud, "y", field_error);
  if (!field_error.empty()) {result.error = field_error; return result;}
  const PointField * z = findField(cloud, "z", field_error);
  if (!field_error.empty()) {result.error = field_error; return result;}
  const PointField * intensity = findField(cloud, "intensity", field_error);
  if (!field_error.empty()) {result.error = field_error; return result;}
  const PointField * timestamp = findField(cloud, "timestamp", field_error);
  if (!field_error.empty()) {result.error = field_error; return result;}
  const PointField * ring = findField(cloud, "ring", field_error);
  if (!field_error.empty()) {result.error = field_error; return result;}

  if (!checkField(x, "x", PointField::FLOAT32, sizeof(float), cloud.point_step, result.error) ||
    !checkField(y, "y", PointField::FLOAT32, sizeof(float), cloud.point_step, result.error) ||
    !checkField(z, "z", PointField::FLOAT32, sizeof(float), cloud.point_step, result.error) ||
    !checkField(
      intensity, "intensity", PointField::FLOAT32, sizeof(float), cloud.point_step,
      result.error) ||
    !checkField(
      timestamp, "timestamp", PointField::FLOAT64, sizeof(double), cloud.point_step,
      result.error) ||
    !checkField(
      ring, "ring", PointField::UINT16, sizeof(std::uint16_t), cloud.point_step,
      result.error))
  {
    return result;
  }

  const std::size_t width = cloud.width;
  const std::size_t height = cloud.height;
  if (height != 0U && width > std::numeric_limits<std::size_t>::max() / height) {
    result.error = "PointCloud2 point count overflows size_t";
    return result;
  }
  result.input_points = width * height;
  result.points.reserve(result.input_points);

  std::uint64_t firing_group = 0U;
  bool have_previous_ring = false;
  std::uint16_t previous_ring = 0U;
  for (std::size_t row = 0U; row < height; ++row) {
    const std::size_t row_offset = row * cloud.row_step;
    for (std::size_t column = 0U; column < width; ++column) {
      const std::size_t point_offset = row_offset + column * cloud.point_step;
      const std::uint8_t * point = cloud.data.data() + point_offset;
      const std::uint16_t ring_value =
        readScalar<std::uint16_t>(point + ring->offset, cloud.is_bigendian);

      if (ring_value < 16U) {
        if (have_previous_ring && ring_value <= previous_ring) {
          ++firing_group;
        }
        previous_ring = ring_value;
        have_previous_ring = true;
      }

      const float x_value = readScalar<float>(point + x->offset, cloud.is_bigendian);
      const float y_value = readScalar<float>(point + y->offset, cloud.is_bigendian);
      const float z_value = readScalar<float>(point + z->offset, cloud.is_bigendian);
      const float intensity_value =
        readScalar<float>(point + intensity->offset, cloud.is_bigendian);
      const double timestamp_seconds =
        readScalar<double>(point + timestamp->offset, cloud.is_bigendian);
      std::uint64_t point_stamp_ns = 0U;
      std::uint64_t adjusted_point_stamp_ns = 0U;

      const bool valid =
        ring_value < 16U && std::isfinite(x_value) && std::isfinite(y_value) &&
        std::isfinite(z_value) && std::isfinite(intensity_value) &&
        secondsToNanoseconds(timestamp_seconds, point_stamp_ns) &&
        point_stamp_ns != 0U &&
        absoluteDifference(point_stamp_ns, header_stamp_ns) <=
        options.max_point_header_delta_ns &&
        addSignedOffset(point_stamp_ns, options.time_offset_ns, adjusted_point_stamp_ns);
      if (!valid) {
        ++result.invalid_points;
        continue;
      }

      result.points.push_back(
        HesaiPoint{
          adjusted_point_stamp_ns, x_value, y_value, z_value, intensity_value,
          ring_value, firing_group});
    }
  }

  result.partial = result.invalid_points != 0U;
  result.ok = true;
  return result;
}

}  // namespace fastlio_go2w_fusion
