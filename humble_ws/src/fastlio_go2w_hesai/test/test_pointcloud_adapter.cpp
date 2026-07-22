#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <sensor_msgs/msg/point_field.hpp>

#include "fastlio_go2w_hesai/pointcloud_adapter.hpp"

namespace
{

using fastlio_go2w_hesai::AdapterConfig;
using fastlio_go2w_hesai::PointcloudAdapter;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

struct InputPoint
{
  float x{1.0F};
  float y{2.0F};
  float z{3.0F};
  float intensity{4.0F};
  double timestamp{100.0};
  std::uint16_t ring{0};
};

bool host_is_big_endian()
{
  const std::uint16_t marker = 0x0102;
  return *reinterpret_cast<const std::uint8_t *>(&marker) == 0x01;
}

template<typename T>
void write_value(std::uint8_t * destination, T value, bool big_endian)
{
  std::array<std::uint8_t, sizeof(T)> bytes{};
  std::memcpy(bytes.data(), &value, sizeof(T));
  if (host_is_big_endian() != big_endian) {
    std::reverse(bytes.begin(), bytes.end());
  }
  std::memcpy(destination, bytes.data(), sizeof(T));
}

template<typename T>
T read_little(const std::uint8_t * source)
{
  std::array<std::uint8_t, sizeof(T)> bytes{};
  std::memcpy(bytes.data(), source, sizeof(T));
  if (host_is_big_endian()) {
    std::reverse(bytes.begin(), bytes.end());
  }
  T value{};
  std::memcpy(&value, bytes.data(), sizeof(T));
  return value;
}

PointField field(const std::string & name, std::uint32_t offset, std::uint8_t datatype)
{
  PointField value;
  value.name = name;
  value.offset = offset;
  value.datatype = datatype;
  value.count = 1;
  return value;
}

PointCloud2 cloud(
  const std::vector<InputPoint> & points, bool big_endian = false,
  std::uint32_t height = 1, std::uint32_t row_padding = 0)
{
  PointCloud2 message;
  message.header.frame_id = "hesai_lidar";
  message.header.stamp.sec = 100;
  message.header.stamp.nanosec = 0;
  message.height = height;
  message.width = height == 0 ? 0 : static_cast<std::uint32_t>(points.size()) / height;
  message.fields = {
    field("x", 0, PointField::FLOAT32),
    field("y", 4, PointField::FLOAT32),
    field("z", 8, PointField::FLOAT32),
    field("intensity", 12, PointField::FLOAT32),
    field("timestamp", 16, PointField::FLOAT64),
    field("ring", 24, PointField::UINT16)};
  message.is_bigendian = big_endian;
  message.point_step = 30;
  message.row_step = message.width * message.point_step + row_padding;
  message.data.assign(static_cast<std::size_t>(message.row_step) * height, 0xA5);
  for (std::size_t index = 0; index < points.size(); ++index) {
    const auto row = static_cast<std::uint32_t>(index) / message.width;
    const auto column = static_cast<std::uint32_t>(index) % message.width;
    auto * base = message.data.data() + row * message.row_step + column * message.point_step;
    write_value(base + 0, points[index].x, big_endian);
    write_value(base + 4, points[index].y, big_endian);
    write_value(base + 8, points[index].z, big_endian);
    write_value(base + 12, points[index].intensity, big_endian);
    write_value(base + 16, points[index].timestamp, big_endian);
    write_value(base + 24, points[index].ring, big_endian);
  }
  return message;
}

AdapterConfig config(double offset = 0.0, std::size_t minimum_points = 1)
{
  AdapterConfig value;
  value.lidar_time_offset_sec = offset;
  value.minimum_points = minimum_points;
  return value;
}

float output_float(const PointCloud2 & message, std::size_t point, std::size_t offset)
{
  return read_little<float>(message.data.data() + point * message.point_step + offset);
}

std::uint16_t output_ring(const PointCloud2 & message, std::size_t point)
{
  return read_little<std::uint16_t>(message.data.data() + point * message.point_step + 20);
}

TEST(PointcloudAdapter, ConvertsSortsAndPreservesVelodyneFields)
{
  PointcloudAdapter adapter(config(0.005));
  auto input = cloud({
      {3.0F, 0.0F, 0.0F, 30.0F, 100.030, 3},
      {1.0F, 0.0F, 0.0F, 10.0F, 100.010, 1},
      {2.0F, 0.0F, 0.0F, 20.0F, 100.010, 2}});

  const auto result = adapter.convert(input);

  ASSERT_TRUE(result.converted);
  EXPECT_EQ(result.cloud.header.frame_id, "hesai_lidar");
  EXPECT_EQ(result.cloud.header.stamp.sec, 100);
  EXPECT_EQ(result.cloud.header.stamp.nanosec, 5000000U);
  EXPECT_EQ(result.cloud.width, 3U);
  EXPECT_EQ(result.cloud.height, 1U);
  EXPECT_FALSE(result.cloud.is_bigendian);
  EXPECT_EQ(result.cloud.point_step, 22U);
  ASSERT_EQ(result.cloud.fields.size(), 6U);
  const std::array<std::string, 6> names{"x", "y", "z", "intensity", "time", "ring"};
  for (std::size_t index = 0; index < names.size(); ++index) {
    EXPECT_EQ(result.cloud.fields[index].name, names[index]);
    EXPECT_EQ(result.cloud.fields[index].count, 1U);
  }
  EXPECT_FLOAT_EQ(output_float(result.cloud, 0, 0), 1.0F);
  EXPECT_FLOAT_EQ(output_float(result.cloud, 1, 0), 2.0F);  // stable equal-time order
  EXPECT_FLOAT_EQ(output_float(result.cloud, 2, 0), 3.0F);
  EXPECT_NEAR(output_float(result.cloud, 0, 16), 0.010, 1e-5);
  EXPECT_NEAR(output_float(result.cloud, 2, 16), 0.030, 1e-5);
  EXPECT_EQ(output_ring(result.cloud, 0), 1U);
  EXPECT_EQ(output_ring(result.cloud, 1), 2U);
  EXPECT_NEAR(result.scan_width_sec, 0.020, 1e-9);
  EXPECT_EQ(adapter.stats().received_frames, 1U);
  EXPECT_EQ(adapter.stats().converted_frames, 1U);
}

TEST(PointcloudAdapter, AppliesNegativeHeaderOffsetWithoutChangingPointTime)
{
  PointcloudAdapter adapter(config(-0.005));
  const auto result = adapter.convert(cloud({{1, 2, 3, 4, 100.025, 4}}));

  ASSERT_TRUE(result.converted);
  EXPECT_EQ(result.cloud.header.stamp.sec, 99);
  EXPECT_EQ(result.cloud.header.stamp.nanosec, 995000000U);
  EXPECT_NEAR(output_float(result.cloud, 0, 16), 0.025, 1e-5);
}

TEST(PointcloudAdapter, HandlesBigEndianAndRowPadding)
{
  PointcloudAdapter adapter(config());
  const auto result = adapter.convert(cloud({
      {1, 2, 3, 4, 100.001, 0},
      {5, 6, 7, 8, 100.002, 1},
      {9, 10, 11, 12, 100.003, 2},
      {13, 14, 15, 16, 100.004, 3}}, true, 2, 11));

  ASSERT_TRUE(result.converted);
  EXPECT_EQ(result.cloud.width, 4U);
  EXPECT_FLOAT_EQ(output_float(result.cloud, 3, 0), 13.0F);
  EXPECT_FLOAT_EQ(output_float(result.cloud, 3, 12), 16.0F);
  EXPECT_EQ(output_ring(result.cloud, 3), 3U);
}

TEST(PointcloudAdapter, RejectsMissingDuplicateAndWrongTypeFields)
{
  {
    PointcloudAdapter adapter(config());
    auto input = cloud({{}});
    input.fields.erase(input.fields.begin() + 4);
    EXPECT_EQ(adapter.convert(input).drop_reason, "schema");
    EXPECT_EQ(adapter.stats().dropped_schema, 1U);
  }
  {
    PointcloudAdapter adapter(config());
    auto input = cloud({{}});
    input.fields.push_back(input.fields.front());
    EXPECT_EQ(adapter.convert(input).drop_reason, "schema");
  }
  {
    PointcloudAdapter adapter(config());
    auto input = cloud({{}});
    input.fields[4].datatype = PointField::FLOAT32;
    EXPECT_EQ(adapter.convert(input).drop_reason, "schema");
  }
}

TEST(PointcloudAdapter, RejectsInvalidLayouts)
{
  {
    PointcloudAdapter adapter(config());
    auto input = cloud({{}});
    input.row_step = input.width * input.point_step - 1;
    EXPECT_EQ(adapter.convert(input).drop_reason, "layout");
  }
  {
    PointcloudAdapter adapter(config());
    auto input = cloud({{}});
    input.data.pop_back();
    EXPECT_EQ(adapter.convert(input).drop_reason, "layout");
  }
  {
    PointcloudAdapter adapter(config());
    auto input = cloud({{}});
    input.fields[5].offset = input.point_step - 1;
    EXPECT_EQ(adapter.convert(input).drop_reason, "schema");
  }
}

TEST(PointcloudAdapter, RejectsNonfiniteOrOutOfRangeTimestampForWholeFrame)
{
  {
    PointcloudAdapter adapter(config());
    auto input = cloud({
        {std::numeric_limits<float>::quiet_NaN(), 2, 3, 4, 100.01, 0},
        {1, 2, 3, 4, std::numeric_limits<double>::quiet_NaN(), 0}});
    EXPECT_EQ(adapter.convert(input).drop_reason, "nonfinite_timestamp");
    EXPECT_EQ(adapter.stats().converted_frames, 0U);
    EXPECT_EQ(adapter.stats().dropped_nonfinite_timestamp, 1U);
    EXPECT_EQ(adapter.stats().invalid_points, 1U);
  }
  {
    PointcloudAdapter adapter(config());
    auto input = cloud({{1, 2, 3, 4, 100.200001, 0}});
    EXPECT_EQ(adapter.convert(input).drop_reason, "timestamp_out_of_range");
    EXPECT_EQ(adapter.stats().dropped_timestamp_out_of_range, 1U);
  }
}

TEST(PointcloudAdapter, FiltersInvalidPointsAndRejectsTooSmallRemainder)
{
  PointcloudAdapter adapter(config(0.0, 2));
  auto input = cloud({
      {1, 2, 3, 4, 100.01, 0},
      {std::numeric_limits<float>::quiet_NaN(), 2, 3, 4, 100.02, 1},
      {1, 2, 3, std::numeric_limits<float>::infinity(), 100.03, 2},
      {1, 2, 3, 4, 100.04, 16}});

  const auto result = adapter.convert(input);

  EXPECT_FALSE(result.converted);
  EXPECT_EQ(result.drop_reason, "too_few_points");
  EXPECT_EQ(result.invalid_points, 3U);
  EXPECT_EQ(adapter.stats().invalid_points, 3U);
  EXPECT_EQ(adapter.stats().dropped_too_few_points, 1U);
}

TEST(PointcloudAdapter, RejectsEmptyAndTinyClouds)
{
  PointcloudAdapter adapter(config(0.0, 3));
  EXPECT_EQ(adapter.convert(cloud({})).drop_reason, "too_few_points");
  EXPECT_EQ(adapter.convert(cloud({{}, {}})).drop_reason, "too_few_points");
  EXPECT_EQ(adapter.stats().dropped_too_few_points, 2U);
}

TEST(PointcloudAdapter, RejectsHeaderRegressionAndCountsReason)
{
  PointcloudAdapter adapter(config());
  ASSERT_TRUE(adapter.convert(cloud({{}})).converted);
  auto regressed = cloud({{}});
  regressed.header.stamp.sec = 99;

  const auto result = adapter.convert(regressed);

  EXPECT_EQ(result.drop_reason, "header_regression");
  EXPECT_EQ(adapter.stats().received_frames, 2U);
  EXPECT_EQ(adapter.stats().converted_frames, 1U);
  EXPECT_EQ(adapter.stats().dropped_header_regression, 1U);
}

TEST(PointcloudAdapter, RejectedFutureHeaderDoesNotPoisonRegressionHistory)
{
  PointcloudAdapter adapter(config());
  ASSERT_TRUE(adapter.convert(cloud({{1, 2, 3, 4, 100.01, 0}})).converted);

  auto corrupt_future = cloud({{1, 2, 3, 4, 100.11, 0}});
  corrupt_future.header.stamp.sec = 107;
  corrupt_future.header.stamp.nanosec = 200000000U;
  EXPECT_EQ(adapter.convert(corrupt_future).drop_reason, "timestamp_out_of_range");

  auto next_normal = cloud({{1, 2, 3, 4, 100.21, 0}});
  next_normal.header.stamp.nanosec = 200000000U;
  const auto result = adapter.convert(next_normal);

  EXPECT_TRUE(result.converted);
  EXPECT_EQ(adapter.stats().converted_frames, 2U);
  EXPECT_EQ(adapter.stats().dropped_timestamp_out_of_range, 1U);
  EXPECT_EQ(adapter.stats().dropped_header_regression, 0U);
}

TEST(PointcloudAdapter, RejectsInvalidHeaderAndInvalidShift)
{
  {
    PointcloudAdapter adapter(config());
    auto input = cloud({{}});
    input.header.stamp.nanosec = 1000000000U;
    EXPECT_EQ(adapter.convert(input).drop_reason, "invalid_header");
  }
  {
    PointcloudAdapter adapter(config(-1.0));
    auto input = cloud({{1, 2, 3, 4, 0.01, 0}});
    input.header.stamp.sec = 0;
    EXPECT_EQ(adapter.convert(input).drop_reason, "invalid_shifted_header");
  }
}

}  // namespace
