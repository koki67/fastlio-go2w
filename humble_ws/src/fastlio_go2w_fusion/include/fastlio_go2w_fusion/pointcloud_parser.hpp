// Copyright 2026 Koki Tanaka
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef FASTLIO_GO2W_FUSION__POINTCLOUD_PARSER_HPP_
#define FASTLIO_GO2W_FUSION__POINTCLOUD_PARSER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "fastlio_go2w_fusion/fusion_core.hpp"

namespace fastlio_go2w_fusion
{

struct HesaiParseOptions
{
  std::int64_t time_offset_ns{0};
  std::uint64_t max_point_header_delta_ns{1000000000ULL};
};

struct HesaiParseResult
{
  bool ok{false};
  bool partial{false};
  std::string error;
  std::uint64_t adjusted_header_stamp_ns{0U};
  std::size_t input_points{0U};
  std::size_t invalid_points{0U};
  std::vector<HesaiPoint> points;
};

// Parses the exact PointXYZIT schema emitted by go2w-hesai-lidar-driver:
// float32 x/y/z/intensity, float64 absolute timestamp, uint16 ring.
// Invalid individual points are counted and skipped; malformed cloud layouts
// fail the complete cloud with a descriptive error.
HesaiParseResult parseHesaiPointCloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const HesaiParseOptions & options = HesaiParseOptions{});

}  // namespace fastlio_go2w_fusion

#endif  // FASTLIO_GO2W_FUSION__POINTCLOUD_PARSER_HPP_
