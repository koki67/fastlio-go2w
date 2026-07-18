// Copyright 2026 Koki Tanaka
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef FASTLIO_GO2W_FUSION__FUSION_CORE_HPP_
#define FASTLIO_GO2W_FUSION__FUSION_CORE_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace fastlio_go2w_fusion
{

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quaternion
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct RigidTransform
{
  Vec3 translation;
  Quaternion rotation;
};

struct FusionOptions
{
  std::size_t mid_point_stride{3U};
  std::size_t hesai_firing_stride{3U};
  double min_range_m{0.5};
  std::size_t max_pending_mid_frames{32U};
  RigidTransform hesai_to_livox;
};

enum class PointSource : std::uint8_t
{
  kMid360 = 0U,
  kHesai = 1U,
};

struct FusedPoint
{
  std::uint32_t offset_time{0U};
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  std::uint8_t reflectivity{0U};
  std::uint8_t tag{0U};
  std::uint8_t line{0U};
  PointSource source{PointSource::kMid360};
};

struct HesaiPoint
{
  std::uint64_t timestamp_ns{0U};
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
  std::uint16_t ring{0U};
  // Zero-based within the source cloud. All rings from a firing share this id.
  std::uint64_t firing_group{0U};
};

struct MidFrame
{
  std::uint64_t header_stamp_ns{0U};
  std::uint64_t timebase_ns{0U};
  std::uint32_t max_offset_ns{0U};
  std::string frame_id;
  std::uint8_t lidar_id{0U};
  std::array<std::uint8_t, 3U> reserved{{0U, 0U, 0U}};
  std::vector<FusedPoint> points;
};

struct HesaiPushReport
{
  bool accepted{false};
  bool nonmonotonic{false};
  std::size_t points_added{0U};
};

enum class MidOnlyReason : std::uint8_t
{
  kNone = 0U,
  kQueueOverflow = 1U,
  kFlush = 2U,
};

struct FusionStats
{
  std::size_t mid_points_input{0U};
  std::size_t mid_points_output{0U};
  std::size_t hesai_points_output{0U};
  std::size_t hesai_points_stale{0U};
  std::size_t hesai_points_filtered{0U};
  bool mid_only_fallback{false};
  MidOnlyReason mid_only_reason{MidOnlyReason::kNone};
};

struct FusionResult
{
  MidFrame frame;
  FusionStats stats;
};

// ROS-independent temporal fusion engine. Accepted Hesai points are kept in
// timestamp order. Once a MID window is emitted, every Hesai point at or before
// its end is removed, whether selected, filtered, or stale.
class FusionCore
{
public:
  explicit FusionCore(const FusionOptions & options);

  HesaiPushReport pushHesaiCloud(
    std::uint64_t cloud_stamp_ns, std::vector<HesaiPoint> points);

  // Returns false for a non-increasing MID timebase. Such a frame is not queued.
  bool enqueueMid(MidFrame frame);

  // Emits every front-of-queue frame with complete Hesai time coverage. When
  // the queue exceeds its bound, the oldest frame is emitted MID-only.
  std::vector<FusionResult> drainReady();

  // Intended for orderly end-of-bag shutdown and deterministic tests.
  std::vector<FusionResult> flushPendingMidOnly();

  std::size_t pendingMidCount() const;
  std::size_t bufferedHesaiPointCount() const;
  bool hasHesaiCoverage(std::uint64_t window_end_ns) const;

private:
  FusionResult fuseFront(MidOnlyReason mid_only_reason);
  void compactHesaiBuffer();

  FusionOptions options_;
  std::array<double, 9U> rotation_matrix_;
  std::vector<HesaiPoint> hesai_points_;
  std::size_t hesai_head_{0U};
  std::deque<MidFrame> pending_mid_;
  bool have_hesai_stamp_{false};
  std::uint64_t last_hesai_stamp_ns_{0U};
  std::uint64_t latest_hesai_coverage_ns_{0U};
  bool have_mid_timebase_{false};
  std::uint64_t last_mid_timebase_ns_{0U};
};

std::array<double, 9U> quaternionToRotationMatrix(const Quaternion & quaternion);
Vec3 transformPoint(
  const std::array<double, 9U> & rotation, const Vec3 & translation,
  const Vec3 & point);
std::uint8_t intensityToReflectivity(float intensity);
bool pointIsFiniteAndInRange(float x, float y, float z, double min_range_m);
std::vector<FusedPoint> selectMidPoints(
  const std::vector<FusedPoint> & points, std::size_t stride, double min_range_m);

}  // namespace fastlio_go2w_fusion

#endif  // FASTLIO_GO2W_FUSION__FUSION_CORE_HPP_
