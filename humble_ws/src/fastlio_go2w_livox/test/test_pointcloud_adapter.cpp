#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_field.hpp>

#include "fastlio_go2w_livox/pointcloud_adapter.hpp"

namespace
{

using fastlio_go2w_livox::AdapterConfig;
using fastlio_go2w_livox::PointcloudAdapter;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

constexpr std::int64_t kBaseNanoseconds = 100000000000LL;

bool host_is_big_endian()
{
  const std::uint16_t marker = 0x0102;
  return *reinterpret_cast<const std::uint8_t *>(&marker) == 0x01;
}

template<typename T>
void write_scalar(std::uint8_t * destination, T value, bool big_endian)
{
  std::array<std::uint8_t, sizeof(T)> bytes{};
  std::memcpy(bytes.data(), &value, sizeof(T));
  if (big_endian != host_is_big_endian()) {
    std::reverse(bytes.begin(), bytes.end());
  }
  std::memcpy(destination, bytes.data(), sizeof(T));
}

PointField field(const std::string & name, std::uint32_t offset, std::uint8_t datatype)
{
  PointField result;
  result.name = name;
  result.offset = offset;
  result.datatype = datatype;
  result.count = 1;
  return result;
}

struct InputPoint
{
  float x;
  float y;
  float z;
  float intensity;
  std::uint8_t tag;
  std::uint8_t line;
  double timestamp_ns;
};

PointCloud2 make_cloud(
  const std::vector<InputPoint> & points,
  bool big_endian = false,
  std::uint32_t height = 1,
  std::uint32_t point_step = 26,
  std::uint32_t row_padding = 0,
  std::int64_t header_ns = kBaseNanoseconds)
{
  if (points.empty() || points.size() % height != 0) {
    throw std::invalid_argument("points must divide evenly into nonempty rows");
  }
  PointCloud2 cloud;
  cloud.header.frame_id = "livox_frame";
  cloud.header.stamp.sec = static_cast<std::int32_t>(header_ns / 1000000000LL);
  cloud.header.stamp.nanosec = static_cast<std::uint32_t>(header_ns % 1000000000LL);
  cloud.height = height;
  cloud.width = static_cast<std::uint32_t>(points.size() / height);
  cloud.fields = {
    field("x", 0, PointField::FLOAT32),
    field("y", 4, PointField::FLOAT32),
    field("z", 8, PointField::FLOAT32),
    field("intensity", 12, PointField::FLOAT32),
    field("tag", 16, PointField::UINT8),
    field("line", 17, PointField::UINT8),
    field("timestamp", 18, PointField::FLOAT64),
  };
  cloud.is_bigendian = big_endian;
  cloud.point_step = point_step;
  cloud.row_step = cloud.width * point_step + row_padding;
  cloud.data.assign(static_cast<std::size_t>(cloud.row_step) * cloud.height, 0xA5);
  cloud.is_dense = true;

  std::size_t index = 0;
  for (std::uint32_t row = 0; row < cloud.height; ++row) {
    for (std::uint32_t column = 0; column < cloud.width; ++column, ++index) {
      auto * base = cloud.data.data() + static_cast<std::size_t>(row) * cloud.row_step +
        static_cast<std::size_t>(column) * cloud.point_step;
      const auto & point = points[index];
      write_scalar(base + 0, point.x, big_endian);
      write_scalar(base + 4, point.y, big_endian);
      write_scalar(base + 8, point.z, big_endian);
      write_scalar(base + 12, point.intensity, big_endian);
      write_scalar(base + 16, point.tag, big_endian);
      write_scalar(base + 17, point.line, big_endian);
      write_scalar(base + 18, point.timestamp_ns, big_endian);
    }
  }
  return cloud;
}

InputPoint valid_point(double offset_ns = 1000.0)
{
  return {1.25F, -2.5F, 3.75F, 42.0F, 0x31, 2,
    static_cast<double>(kBaseNanoseconds) + offset_ns};
}

PointcloudAdapter adapter(std::size_t minimum_points = 1, double max_delta_sec = 0.2)
{
  AdapterConfig config;
  config.minimum_points = minimum_points;
  config.max_point_header_delta_sec = max_delta_sec;
  return PointcloudAdapter(config);
}

TEST(PointcloudAdapter, ReconstructsDriverStyleTwentySixByteLayout)
{
  auto converter = adapter();
  const auto input = make_cloud({valid_point(1024.0), valid_point(2048.0)});
  const auto result = converter.convert(input);

  ASSERT_TRUE(result.converted) << result.drop_reason;
  EXPECT_EQ(result.message.header, input.header);
  EXPECT_EQ(result.message.timebase, static_cast<std::uint64_t>(kBaseNanoseconds));
  EXPECT_EQ(result.message.point_num, 2U);
  EXPECT_EQ(result.message.lidar_id, 0U);
  EXPECT_EQ(result.message.rsvd, (std::array<std::uint8_t, 3>{0, 0, 0}));
  ASSERT_EQ(result.message.points.size(), 2U);
  EXPECT_EQ(result.message.points[0].offset_time, 1024U);
  EXPECT_FLOAT_EQ(result.message.points[0].x, 1.25F);
  EXPECT_FLOAT_EQ(result.message.points[0].y, -2.5F);
  EXPECT_FLOAT_EQ(result.message.points[0].z, 3.75F);
  EXPECT_EQ(result.message.points[0].reflectivity, 42U);
  EXPECT_EQ(result.message.points[0].tag, 0x31U);
  EXPECT_EQ(result.message.points[0].line, 2U);
  EXPECT_DOUBLE_EQ(result.scan_width_sec, 1024.0e-9);
}

TEST(PointcloudAdapter, SupportsOrganizedPaddedBigEndianClouds)
{
  auto converter = adapter();
  auto first = valid_point(1000.0);
  auto second = valid_point(2000.0);
  auto third = valid_point(3000.0);
  auto fourth = valid_point(4000.0);
  fourth.x = 9.0F;
  fourth.intensity = 255.0F;
  const auto input = make_cloud(
    {first, second, third, fourth}, true, 2, 32, 12);

  const auto result = converter.convert(input);
  ASSERT_TRUE(result.converted) << result.drop_reason;
  ASSERT_EQ(result.message.points.size(), 4U);
  EXPECT_FLOAT_EQ(result.message.points.back().x, 9.0F);
  EXPECT_EQ(result.message.points.back().reflectivity, 255U);
  EXPECT_EQ(result.message.points.back().offset_time, 4000U);
}

TEST(PointcloudAdapter, StableSortsTimestampsAndPreservesTies)
{
  auto converter = adapter();
  auto late = valid_point(3000.0);
  late.x = 3.0F;
  auto first_tie = valid_point(1000.0);
  first_tie.x = 1.0F;
  auto second_tie = valid_point(1000.0);
  second_tie.x = 2.0F;

  const auto result = converter.convert(make_cloud({late, first_tie, second_tie}));
  ASSERT_TRUE(result.converted) << result.drop_reason;
  EXPECT_TRUE(result.reordered);
  ASSERT_EQ(result.message.points.size(), 3U);
  EXPECT_FLOAT_EQ(result.message.points[0].x, 1.0F);
  EXPECT_FLOAT_EQ(result.message.points[1].x, 2.0F);
  EXPECT_FLOAT_EQ(result.message.points[2].x, 3.0F);
  EXPECT_EQ(converter.stats().reordered_frames, 1U);
}

TEST(PointcloudAdapter, RoundsEpochScaleDoubleBeforeSubtractingHeader)
{
  constexpr std::int64_t epoch_header_ns = 1785119002244494983LL;
  const double encoded = static_cast<double>(epoch_header_ns + 1000LL);
  const auto expected = static_cast<std::uint32_t>(
    static_cast<std::int64_t>(std::llround(encoded)) - epoch_header_ns);
  auto point = valid_point();
  point.timestamp_ns = encoded;
  auto converter = adapter();

  const auto result = converter.convert(make_cloud({point}, false, 1, 26, 0, epoch_header_ns));
  ASSERT_TRUE(result.converted) << result.drop_reason;
  EXPECT_EQ(result.message.points[0].offset_time, expected);
}

TEST(PointcloudAdapter, ClampsOnlyNegativeOffsetsCausedByFloat64Quantization)
{
  constexpr std::int64_t epoch_header_ns = 1785119002164648506LL;
  auto quantized = valid_point();
  quantized.timestamp_ns = static_cast<double>(epoch_header_ns);
  auto quantized_converter = adapter();
  const auto accepted = quantized_converter.convert(
    make_cloud({quantized}, false, 1, 26, 0, epoch_header_ns));
  ASSERT_TRUE(accepted.converted) << accepted.drop_reason;
  EXPECT_EQ(accepted.message.points[0].offset_time, 0U);
  EXPECT_EQ(quantized_converter.stats().quantization_clamped_frames, 1U);
  EXPECT_EQ(quantized_converter.stats().quantization_clamped_points, 1U);

  auto true_negative = valid_point();
  true_negative.timestamp_ns = static_cast<double>(epoch_header_ns - 1000LL);
  auto negative_converter = adapter();
  const auto rejected = negative_converter.convert(
    make_cloud({true_negative}, false, 1, 26, 0, epoch_header_ns));
  EXPECT_FALSE(rejected.converted);
  EXPECT_EQ(rejected.drop_reason, "negative_offset");
}

TEST(PointcloudAdapter, AcceptsUint32BoundaryAndRejectsOverflow)
{
  const double maximum_delta_sec =
    static_cast<double>(std::numeric_limits<std::uint32_t>::max()) / 1.0e9;
  auto boundary_converter = adapter(1, maximum_delta_sec);
  auto boundary = valid_point();
  boundary.timestamp_ns = static_cast<double>(kBaseNanoseconds) +
    static_cast<double>(std::numeric_limits<std::uint32_t>::max());
  const auto valid = boundary_converter.convert(make_cloud({boundary}));
  ASSERT_TRUE(valid.converted) << valid.drop_reason;
  EXPECT_EQ(valid.message.points[0].offset_time, std::numeric_limits<std::uint32_t>::max());

  auto overflow_converter = adapter(1, maximum_delta_sec);
  auto overflow = valid_point();
  overflow.timestamp_ns = static_cast<double>(kBaseNanoseconds) + 4294967296.0;
  const auto invalid = overflow_converter.convert(make_cloud({overflow}));
  EXPECT_FALSE(invalid.converted);
  EXPECT_EQ(invalid.drop_reason, "offset_too_large");
}

TEST(PointcloudAdapter, RejectsSchemaAndLayoutAnomalies)
{
  auto schema_converter = adapter();
  auto missing_field = make_cloud({valid_point()});
  missing_field.fields.pop_back();
  EXPECT_EQ(schema_converter.convert(missing_field).drop_reason, "schema");

  auto duplicate_converter = adapter();
  auto duplicate_field = make_cloud({valid_point()});
  duplicate_field.fields.push_back(duplicate_field.fields.front());
  EXPECT_EQ(duplicate_converter.convert(duplicate_field).drop_reason, "schema");

  auto count_converter = adapter();
  auto invalid_count = make_cloud({valid_point()});
  invalid_count.fields.front().count = 2;
  EXPECT_EQ(count_converter.convert(invalid_count).drop_reason, "schema");

  auto overlap_converter = adapter();
  auto overlapping_fields = make_cloud({valid_point()});
  overlapping_fields.fields.back().offset = 16;
  EXPECT_EQ(overlap_converter.convert(overlapping_fields).drop_reason, "schema");

  auto layout_converter = adapter();
  auto short_row = make_cloud({valid_point()});
  short_row.row_step = short_row.point_step - 1;
  EXPECT_EQ(layout_converter.convert(short_row).drop_reason, "layout");

  auto data_converter = adapter();
  auto short_data = make_cloud({valid_point()});
  short_data.data.pop_back();
  EXPECT_EQ(data_converter.convert(short_data).drop_reason, "layout");
}

TEST(PointcloudAdapter, RejectsInvalidAndRegressingHeaders)
{
  auto invalid_converter = adapter();
  auto zero_header = make_cloud({valid_point()});
  zero_header.header.stamp.sec = 0;
  zero_header.header.stamp.nanosec = 0;
  EXPECT_EQ(invalid_converter.convert(zero_header).drop_reason, "invalid_header");

  auto regression_converter = adapter();
  auto later_point = valid_point();
  later_point.timestamp_ns += 1000000000.0;
  ASSERT_TRUE(regression_converter.convert(
    make_cloud({later_point}, false, 1, 26, 0, kBaseNanoseconds + 1000000000LL)).converted);
  EXPECT_EQ(
    regression_converter.convert(make_cloud({valid_point()})).drop_reason,
    "header_regression");
}

TEST(PointcloudAdapter, RejectsNonfiniteValues)
{
  auto coordinate_converter = adapter();
  auto coordinate = valid_point();
  coordinate.x = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(
    coordinate_converter.convert(make_cloud({coordinate})).drop_reason,
    "nonfinite_coordinate");

  auto intensity_converter = adapter();
  auto intensity = valid_point();
  intensity.intensity = std::numeric_limits<float>::infinity();
  EXPECT_EQ(
    intensity_converter.convert(make_cloud({intensity})).drop_reason,
    "nonfinite_intensity");

  auto timestamp_converter = adapter();
  auto timestamp = valid_point();
  timestamp.timestamp_ns = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    timestamp_converter.convert(make_cloud({timestamp})).drop_reason,
    "nonfinite_timestamp");
}

TEST(PointcloudAdapter, RejectsInvalidIntensity)
{
  auto range_converter = adapter();
  auto out_of_range = valid_point();
  out_of_range.intensity = 256.0F;
  EXPECT_EQ(
    range_converter.convert(make_cloud({out_of_range})).drop_reason,
    "intensity_out_of_range");

  auto fractional_converter = adapter();
  auto fractional = valid_point();
  fractional.intensity = 1.5F;
  EXPECT_EQ(
    fractional_converter.convert(make_cloud({fractional})).drop_reason,
    "intensity_nonintegral");
}

TEST(PointcloudAdapter, RejectsNegativeAndConfiguredOversizeOffsets)
{
  auto negative_converter = adapter();
  auto negative = valid_point();
  negative.timestamp_ns = static_cast<double>(kBaseNanoseconds - 1);
  EXPECT_EQ(
    negative_converter.convert(make_cloud({negative})).drop_reason,
    "negative_offset");

  auto large_converter = adapter();
  auto large = valid_point();
  large.timestamp_ns = static_cast<double>(kBaseNanoseconds + 200000001LL);
  EXPECT_EQ(
    large_converter.convert(make_cloud({large})).drop_reason,
    "offset_too_large");
}

TEST(PointcloudAdapter, RejectsTooFewPointsAndCountsCriticalDrops)
{
  auto converter = adapter(2);
  const auto result = converter.convert(make_cloud({valid_point()}));
  EXPECT_FALSE(result.converted);
  EXPECT_EQ(result.drop_reason, "too_few_points");
  EXPECT_EQ(converter.stats().received_frames, 1U);
  EXPECT_EQ(converter.stats().converted_frames, 0U);
  EXPECT_EQ(converter.stats().dropped_too_few_points, 1U);
  EXPECT_EQ(converter.stats().critical_drops(), 1U);
}

TEST(PointcloudAdapter, RejectsInvalidConfiguration)
{
  AdapterConfig config;
  config.minimum_points = 0;
  EXPECT_THROW(
    {PointcloudAdapter converter{config}; (void)converter;}, std::invalid_argument);
  config.minimum_points = 1;
  config.max_point_header_delta_sec = std::numeric_limits<double>::infinity();
  EXPECT_THROW(
    {PointcloudAdapter converter{config}; (void)converter;}, std::invalid_argument);
}

}  // namespace
