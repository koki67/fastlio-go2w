// Copyright 2026 Koki Tanaka
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "fastlio_go2w_fusion/fusion_core.hpp"

namespace fusion = fastlio_go2w_fusion;

namespace
{

fusion::FusedPoint midPoint(std::uint32_t offset, float x = 1.0F)
{
  fusion::FusedPoint point;
  point.offset_time = offset;
  point.x = x;
  point.reflectivity = 10U;
  point.line = 0U;
  point.source = fusion::PointSource::kMid360;
  return point;
}

fusion::MidFrame midFrame(std::uint64_t start, std::uint32_t duration)
{
  fusion::MidFrame frame;
  frame.header_stamp_ns = start;
  frame.timebase_ns = start;
  frame.max_offset_ns = duration;
  frame.frame_id = "livox_frame";
  return frame;
}

fusion::HesaiPoint hesaiPoint(
  std::uint64_t stamp, std::uint16_t ring, std::uint64_t group,
  float x = 1.0F, float intensity = 20.0F)
{
  fusion::HesaiPoint point;
  point.timestamp_ns = stamp;
  point.x = x;
  point.intensity = intensity;
  point.ring = ring;
  point.firing_group = group;
  return point;
}

fusion::FusionOptions identityOptions()
{
  fusion::FusionOptions options;
  options.mid_point_stride = 1U;
  options.hesai_firing_stride = 1U;
  options.min_range_m = 0.0;
  options.max_pending_mid_frames = 4U;
  return options;
}

}  // namespace

TEST(FusionMath, NormalizesQuaternionAndAppliesTransform)
{
  const double root_half = std::sqrt(0.5);
  const auto rotation = fusion::quaternionToRotationMatrix(
    fusion::Quaternion{0.0, 0.0, root_half * 2.0, root_half * 2.0});
  const auto output = fusion::transformPoint(
    rotation, fusion::Vec3{1.0, 2.0, 3.0}, fusion::Vec3{2.0, 0.0, 0.0});
  EXPECT_NEAR(output.x, 1.0, 1.0e-9);
  EXPECT_NEAR(output.y, 4.0, 1.0e-9);
  EXPECT_NEAR(output.z, 3.0, 1.0e-9);
  EXPECT_THROW(
    fusion::quaternionToRotationMatrix(fusion::Quaternion{0.0, 0.0, 0.0, 0.0}),
    std::invalid_argument);
}

TEST(FusionMath, ClampsAndRoundsReflectivity)
{
  EXPECT_EQ(fusion::intensityToReflectivity(-1.0F), 0U);
  EXPECT_EQ(fusion::intensityToReflectivity(10.6F), 11U);
  EXPECT_EQ(fusion::intensityToReflectivity(300.0F), 255U);
  EXPECT_EQ(
    fusion::intensityToReflectivity(std::numeric_limits<float>::quiet_NaN()), 0U);
}

TEST(FusionCore, MidStrideMatchesFastLioEligibilityAndRangePhase)
{
  std::vector<fusion::FusedPoint> points{
    midPoint(9U, 0.1F), midPoint(1U, 0.1F), midPoint(2U, 2.0F),
    midPoint(3U, 3.0F), midPoint(4U, 4.0F), midPoint(5U, 5.0F),
    midPoint(6U, 6.0F)};
  points[2].line = 4U;
  points[3].tag = 0x30U;
  const auto selected = fusion::selectMidPoints(points, 2U, 0.5);
  ASSERT_EQ(selected.size(), 3U);
  EXPECT_EQ(selected[0].offset_time, 0U);
  EXPECT_EQ(selected[1].offset_time, 4U);
  EXPECT_EQ(selected[2].offset_time, 6U);
}

TEST(FusionCore, CropsConsumesTransformsMapsAndStableSorts)
{
  auto options = identityOptions();
  options.hesai_to_livox.translation = fusion::Vec3{1.0, 0.0, 0.0};
  fusion::FusionCore core(options);
  std::vector<fusion::HesaiPoint> hesai{
    hesaiPoint(1200U, 3U, 3U), hesaiPoint(1000U, 0U, 0U),
    hesaiPoint(900U, 1U, 0U), hesaiPoint(1050U, 15U, 1U, 2.0F, 999.0F)};
  EXPECT_TRUE(core.pushHesaiCloud(1100U, std::move(hesai)).accepted);

  auto frame = midFrame(1000U, 100U);
  frame.points = {midPoint(50U), midPoint(5U)};
  ASSERT_TRUE(core.enqueueMid(std::move(frame)));
  auto results = core.drainReady();
  ASSERT_EQ(results.size(), 1U);
  const auto & result = results.front();
  EXPECT_EQ(result.stats.hesai_points_stale, 1U);
  EXPECT_EQ(result.stats.hesai_points_output, 2U);
  ASSERT_EQ(result.frame.points.size(), 4U);
  EXPECT_TRUE(
    std::is_sorted(
      result.frame.points.begin(), result.frame.points.end(),
      [](const fusion::FusedPoint & lhs, const fusion::FusedPoint & rhs) {
        return lhs.offset_time < rhs.offset_time;
      }));
  const auto first_hesai = std::find_if(
    result.frame.points.begin(), result.frame.points.end(),
    [](const fusion::FusedPoint & point) {
      return point.source == fusion::PointSource::kHesai;
    });
  ASSERT_NE(first_hesai, result.frame.points.end());
  EXPECT_FLOAT_EQ(first_hesai->x, 2.0F);
  EXPECT_EQ(first_hesai->line, 4U);
  EXPECT_EQ(first_hesai->tag, 0U);
  EXPECT_EQ(result.frame.points.back().line, 19U);
  EXPECT_EQ(result.frame.points.back().reflectivity, 255U);
  EXPECT_EQ(core.bufferedHesaiPointCount(), 1U);

  EXPECT_TRUE(core.pushHesaiCloud(1300U, {}).accepted);
  auto second = midFrame(1101U, 199U);
  ASSERT_TRUE(core.enqueueMid(std::move(second)));
  auto second_results = core.drainReady();
  ASSERT_EQ(second_results.size(), 1U);
  EXPECT_EQ(second_results.front().stats.hesai_points_output, 1U);
  EXPECT_EQ(core.bufferedHesaiPointCount(), 0U);
}

TEST(FusionCore, KeepsAllRingsWithinSelectedFiringGroups)
{
  auto options = identityOptions();
  options.hesai_firing_stride = 2U;
  fusion::FusionCore core(options);
  std::vector<fusion::HesaiPoint> points;
  for (std::uint64_t group = 0U; group < 4U; ++group) {
    for (std::uint16_t ring = 0U; ring < 16U; ++ring) {
      points.push_back(hesaiPoint(1000U + group, ring, group));
    }
  }
  EXPECT_TRUE(core.pushHesaiCloud(1100U, std::move(points)).accepted);
  EXPECT_TRUE(core.enqueueMid(midFrame(1000U, 100U)));
  auto output = core.drainReady();
  ASSERT_EQ(output.size(), 1U);
  EXPECT_EQ(output.front().stats.hesai_points_output, 32U);
  std::array<std::size_t, 16U> rings{};
  for (const auto & point : output.front().frame.points) {
    ++rings[point.line - 4U];
  }
  for (const auto count : rings) {
    EXPECT_EQ(count, 2U);
  }
}

TEST(FusionCore, RejectsTimestampRegressions)
{
  fusion::FusionCore core(identityOptions());
  EXPECT_TRUE(core.pushHesaiCloud(100U, {}).accepted);
  const auto equal = core.pushHesaiCloud(100U, {});
  EXPECT_FALSE(equal.accepted);
  EXPECT_TRUE(equal.nonmonotonic);
  EXPECT_FALSE(core.pushHesaiCloud(99U, {}).accepted);

  EXPECT_TRUE(core.enqueueMid(midFrame(100U, 1U)));
  EXPECT_FALSE(core.enqueueMid(midFrame(100U, 2U)));
}

TEST(FusionCore, BoundedQueueFallsBackMidOnly)
{
  auto options = identityOptions();
  options.max_pending_mid_frames = 1U;
  fusion::FusionCore core(options);
  auto first = midFrame(100U, 10U);
  first.points.push_back(midPoint(1U));
  auto second = midFrame(200U, 10U);
  second.points.push_back(midPoint(1U));
  EXPECT_TRUE(core.enqueueMid(std::move(first)));
  EXPECT_TRUE(core.enqueueMid(std::move(second)));
  const auto ready = core.drainReady();
  ASSERT_EQ(ready.size(), 1U);
  EXPECT_TRUE(ready.front().stats.mid_only_fallback);
  EXPECT_EQ(
    ready.front().stats.mid_only_reason, fusion::MidOnlyReason::kQueueOverflow);
  EXPECT_EQ(ready.front().frame.points.size(), 1U);
  EXPECT_EQ(core.pendingMidCount(), 1U);
  const auto flushed = core.flushPendingMidOnly();
  ASSERT_EQ(flushed.size(), 1U);
  EXPECT_TRUE(flushed.front().stats.mid_only_fallback);
  EXPECT_EQ(
    flushed.front().stats.mid_only_reason, fusion::MidOnlyReason::kFlush);
}
