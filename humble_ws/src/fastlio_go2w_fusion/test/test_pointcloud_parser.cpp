// Copyright 2026 Koki Tanaka
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#include "gtest/gtest.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include "fastlio_go2w_fusion/pointcloud_parser.hpp"

namespace fusion = fastlio_go2w_fusion;

namespace
{

sensor_msgs::msg::PointField field(
  const std::string & name, std::uint32_t offset, std::uint8_t datatype)
{
  sensor_msgs::msg::PointField result;
  result.name = name;
  result.offset = offset;
  result.datatype = datatype;
  result.count = 1U;
  return result;
}

template<typename T>
void write(std::vector<std::uint8_t> & data, std::size_t offset, T value)
{
  std::memcpy(data.data() + offset, &value, sizeof(T));
}

sensor_msgs::msg::PointCloud2 cloudWithTwoFirings()
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp.sec = 2;
  cloud.height = 1U;
  cloud.width = 32U;
  cloud.fields = {
    field("x", 0U, sensor_msgs::msg::PointField::FLOAT32),
    field("y", 4U, sensor_msgs::msg::PointField::FLOAT32),
    field("z", 8U, sensor_msgs::msg::PointField::FLOAT32),
    field("intensity", 12U, sensor_msgs::msg::PointField::FLOAT32),
    field("timestamp", 16U, sensor_msgs::msg::PointField::FLOAT64),
    field("ring", 24U, sensor_msgs::msg::PointField::UINT16)};
  cloud.point_step = 26U;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(cloud.row_step);
  cloud.is_dense = true;
  for (std::uint32_t index = 0U; index < cloud.width; ++index) {
    const std::size_t base = index * cloud.point_step;
    write(cloud.data, base + 0U, 1.0F + index);
    write(cloud.data, base + 4U, 2.0F);
    write(cloud.data, base + 8U, 3.0F);
    write(cloud.data, base + 12U, 42.0F);
    write(cloud.data, base + 16U, 2.0 + static_cast<double>(index / 16U) * 0.01);
    write(cloud.data, base + 24U, static_cast<std::uint16_t>(index % 16U));
  }
  return cloud;
}

}  // namespace

TEST(PointCloudParser, ParsesSchemaAndAssignsFiringGroups)
{
  const auto result = fusion::parseHesaiPointCloud(cloudWithTwoFirings());
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_FALSE(result.partial);
  ASSERT_EQ(result.points.size(), 32U);
  EXPECT_EQ(result.points.front().firing_group, 0U);
  EXPECT_EQ(result.points[15].firing_group, 0U);
  EXPECT_EQ(result.points[16].firing_group, 1U);
  EXPECT_EQ(result.points.back().ring, 15U);
  EXPECT_EQ(result.points.back().timestamp_ns, 2010000000ULL);
}

TEST(PointCloudParser, AppliesSignedTimeOffset)
{
  fusion::HesaiParseOptions options;
  options.time_offset_ns = -5000000;
  const auto result = fusion::parseHesaiPointCloud(cloudWithTwoFirings(), options);
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.adjusted_header_stamp_ns, 1995000000ULL);
  EXPECT_EQ(result.points.front().timestamp_ns, 1995000000ULL);
}

TEST(PointCloudParser, RetainsValidSubsetAndReportsPartialCloud)
{
  auto cloud = cloudWithTwoFirings();
  write(
    cloud.data, static_cast<std::size_t>(5U * cloud.point_step),
    std::numeric_limits<float>::quiet_NaN());
  write(
    cloud.data, static_cast<std::size_t>(20U * cloud.point_step + 24U),
    static_cast<std::uint16_t>(20U));
  const auto result = fusion::parseHesaiPointCloud(cloud);
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_TRUE(result.partial);
  EXPECT_EQ(result.invalid_points, 2U);
  EXPECT_EQ(result.points.size(), 30U);
}

TEST(PointCloudParser, RejectsMissingOrWrongFields)
{
  auto missing = cloudWithTwoFirings();
  missing.fields.erase(missing.fields.begin() + 4);
  const auto missing_result = fusion::parseHesaiPointCloud(missing);
  EXPECT_FALSE(missing_result.ok);
  EXPECT_NE(missing_result.error.find("timestamp"), std::string::npos);

  auto wrong = cloudWithTwoFirings();
  wrong.fields.back().datatype = sensor_msgs::msg::PointField::UINT8;
  const auto wrong_result = fusion::parseHesaiPointCloud(wrong);
  EXPECT_FALSE(wrong_result.ok);
  EXPECT_NE(wrong_result.error.find("datatype"), std::string::npos);
}

TEST(PointCloudParser, RejectsTruncatedRowsAndOutlyingPointTimes)
{
  auto truncated = cloudWithTwoFirings();
  truncated.data.resize(truncated.data.size() - 1U);
  EXPECT_FALSE(fusion::parseHesaiPointCloud(truncated).ok);

  auto outlying = cloudWithTwoFirings();
  write(outlying.data, 16U, 10.0);
  fusion::HesaiParseOptions options;
  options.max_point_header_delta_ns = 100000000ULL;
  const auto result = fusion::parseHesaiPointCloud(outlying, options);
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_TRUE(result.partial);
  EXPECT_EQ(result.invalid_points, 1U);
}
