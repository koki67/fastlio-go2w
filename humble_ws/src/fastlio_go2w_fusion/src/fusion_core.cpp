// Copyright 2026 Koki Tanaka
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "fastlio_go2w_fusion/fusion_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace fastlio_go2w_fusion
{
namespace
{

bool finite(double value)
{
  return std::isfinite(value);
}

std::uint64_t windowEnd(const MidFrame & frame)
{
  const std::uint64_t offset = frame.max_offset_ns;
  if (frame.timebase_ns > std::numeric_limits<std::uint64_t>::max() - offset) {
    return std::numeric_limits<std::uint64_t>::max();
  }
  return frame.timebase_ns + offset;
}

bool hesaiTimeLess(const HesaiPoint & lhs, const HesaiPoint & rhs)
{
  return lhs.timestamp_ns < rhs.timestamp_ns;
}

}  // namespace

std::array<double, 9U> quaternionToRotationMatrix(const Quaternion & quaternion)
{
  if (!finite(quaternion.x) || !finite(quaternion.y) || !finite(quaternion.z) ||
    !finite(quaternion.w))
  {
    throw std::invalid_argument("rotation quaternion must contain finite values");
  }

  const double norm = std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  if (norm < 1.0e-12) {
    throw std::invalid_argument("rotation quaternion norm must be nonzero");
  }

  const double x = quaternion.x / norm;
  const double y = quaternion.y / norm;
  const double z = quaternion.z / norm;
  const double w = quaternion.w / norm;

  return {{
    1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
    2.0 * (x * z + y * w),
    2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
    2.0 * (y * z - x * w),
    2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
    1.0 - 2.0 * (x * x + y * y)}};
}

Vec3 transformPoint(
  const std::array<double, 9U> & rotation, const Vec3 & translation,
  const Vec3 & point)
{
  return Vec3{
    rotation[0] * point.x + rotation[1] * point.y + rotation[2] * point.z +
    translation.x,
    rotation[3] * point.x + rotation[4] * point.y + rotation[5] * point.z +
    translation.y,
    rotation[6] * point.x + rotation[7] * point.y + rotation[8] * point.z +
    translation.z};
}

std::uint8_t intensityToReflectivity(float intensity)
{
  if (!std::isfinite(intensity) || intensity <= 0.0F) {
    return 0U;
  }
  if (intensity >= 255.0F) {
    return 255U;
  }
  return static_cast<std::uint8_t>(std::lround(intensity));
}

bool pointIsFiniteAndInRange(float x, float y, float z, double min_range_m)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
    !finite(min_range_m) || min_range_m < 0.0)
  {
    return false;
  }
  const double squared_range =
    static_cast<double>(x) * x + static_cast<double>(y) * y +
    static_cast<double>(z) * z;
  return squared_range >= min_range_m * min_range_m;
}

std::vector<FusedPoint> selectMidPoints(
  const std::vector<FusedPoint> & points, std::size_t stride, double min_range_m)
{
  if (stride == 0U) {
    throw std::invalid_argument("MID point stride must be at least one");
  }

  if (points.empty()) {
    return {};
  }

  std::vector<FusedPoint> selected;
  selected.reserve(1U + (points.size() - 1U + stride - 1U) / stride);

  // FAST-LIO's Livox path starts at index 1 because index 0 is the
  // acquisition anchor. Preserve that anchor so its second preprocessing pass
  // skips the same point, then reproduce the original valid-point phase.
  FusedPoint anchor = points.front();
  anchor.offset_time = 0U;
  selected.push_back(anchor);

  std::size_t eligible_count = 0U;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    const auto & point = points[index];
    const std::uint8_t return_tag = point.tag & 0x30U;
    const bool accepted_tag = return_tag == 0x10U || return_tag == 0x00U;
    if (point.line >= 4U || !accepted_tag) {
      continue;
    }
    ++eligible_count;
    if (eligible_count % stride == 0U &&
      pointIsFiniteAndInRange(point.x, point.y, point.z, min_range_m))
    {
      selected.push_back(point);
    }
  }
  return selected;
}

FusionCore::FusionCore(const FusionOptions & options)
: options_(options),
  rotation_matrix_(quaternionToRotationMatrix(options.hesai_to_livox.rotation))
{
  if (options_.mid_point_stride == 0U) {
    throw std::invalid_argument("mid_point_stride must be at least one");
  }
  if (options_.hesai_firing_stride == 0U) {
    throw std::invalid_argument("hesai_firing_stride must be at least one");
  }
  if (options_.max_pending_mid_frames == 0U) {
    throw std::invalid_argument("max_pending_mid_frames must be at least one");
  }
  if (!finite(options_.min_range_m) || options_.min_range_m < 0.0) {
    throw std::invalid_argument("min_range_m must be finite and non-negative");
  }
  const auto & translation = options_.hesai_to_livox.translation;
  if (!finite(translation.x) || !finite(translation.y) || !finite(translation.z)) {
    throw std::invalid_argument("transform translation must contain finite values");
  }
}

HesaiPushReport FusionCore::pushHesaiCloud(
  std::uint64_t cloud_stamp_ns, std::vector<HesaiPoint> points)
{
  HesaiPushReport report;
  if (have_hesai_stamp_ && cloud_stamp_ns <= last_hesai_stamp_ns_) {
    report.nonmonotonic = true;
    return report;
  }

  std::stable_sort(points.begin(), points.end(), hesaiTimeLess);
  const std::size_t old_size = hesai_points_.size();
  hesai_points_.insert(
    hesai_points_.end(), std::make_move_iterator(points.begin()),
    std::make_move_iterator(points.end()));
  if (hesai_head_ < old_size && !points.empty()) {
    std::inplace_merge(
      hesai_points_.begin() + static_cast<std::ptrdiff_t>(hesai_head_),
      hesai_points_.begin() + static_cast<std::ptrdiff_t>(old_size),
      hesai_points_.end(), hesaiTimeLess);
  }

  report.accepted = true;
  report.points_added = points.size();
  have_hesai_stamp_ = true;
  last_hesai_stamp_ns_ = cloud_stamp_ns;
  latest_hesai_coverage_ns_ = std::max(latest_hesai_coverage_ns_, cloud_stamp_ns);
  if (hesai_head_ < hesai_points_.size()) {
    latest_hesai_coverage_ns_ =
      std::max(latest_hesai_coverage_ns_, hesai_points_.back().timestamp_ns);
  }
  return report;
}

bool FusionCore::enqueueMid(MidFrame frame)
{
  if (have_mid_timebase_ && frame.timebase_ns <= last_mid_timebase_ns_) {
    return false;
  }
  have_mid_timebase_ = true;
  last_mid_timebase_ns_ = frame.timebase_ns;
  pending_mid_.push_back(std::move(frame));
  return true;
}

std::vector<FusionResult> FusionCore::drainReady()
{
  std::vector<FusionResult> results;
  while (!pending_mid_.empty()) {
    if (hasHesaiCoverage(windowEnd(pending_mid_.front()))) {
      results.push_back(fuseFront(MidOnlyReason::kNone));
      continue;
    }
    if (pending_mid_.size() > options_.max_pending_mid_frames) {
      results.push_back(fuseFront(MidOnlyReason::kQueueOverflow));
      continue;
    }
    break;
  }
  return results;
}

std::vector<FusionResult> FusionCore::flushPendingMidOnly()
{
  std::vector<FusionResult> results;
  results.reserve(pending_mid_.size());
  while (!pending_mid_.empty()) {
    results.push_back(fuseFront(MidOnlyReason::kFlush));
  }
  return results;
}

std::size_t FusionCore::pendingMidCount() const
{
  return pending_mid_.size();
}

std::size_t FusionCore::bufferedHesaiPointCount() const
{
  return hesai_points_.size() - hesai_head_;
}

bool FusionCore::hasHesaiCoverage(std::uint64_t window_end_ns) const
{
  return have_hesai_stamp_ && latest_hesai_coverage_ns_ >= window_end_ns;
}

FusionResult FusionCore::fuseFront(MidOnlyReason mid_only_reason)
{
  const bool include_hesai = mid_only_reason == MidOnlyReason::kNone;
  FusionResult result;
  result.frame = std::move(pending_mid_.front());
  pending_mid_.pop_front();
  result.stats.mid_points_input = result.frame.points.size();
  result.frame.points = selectMidPoints(
    result.frame.points, options_.mid_point_stride, options_.min_range_m);
  result.stats.mid_points_output = result.frame.points.size();
  result.stats.mid_only_fallback = !include_hesai;
  result.stats.mid_only_reason = mid_only_reason;

  const std::uint64_t start_ns = result.frame.timebase_ns;
  const std::uint64_t end_ns = windowEnd(result.frame);
  while (hesai_head_ < hesai_points_.size() &&
    hesai_points_[hesai_head_].timestamp_ns <= end_ns)
  {
    const auto & point = hesai_points_[hesai_head_];
    ++hesai_head_;
    if (!include_hesai || point.timestamp_ns < start_ns) {
      ++result.stats.hesai_points_stale;
      continue;
    }
    if (point.ring >= 16U ||
      ((point.firing_group + 1U) % options_.hesai_firing_stride) != 0U ||
      !pointIsFiniteAndInRange(point.x, point.y, point.z, options_.min_range_m))
    {
      ++result.stats.hesai_points_filtered;
      continue;
    }

    const Vec3 transformed = transformPoint(
      rotation_matrix_, options_.hesai_to_livox.translation,
      Vec3{point.x, point.y, point.z});
    FusedPoint fused;
    fused.offset_time = static_cast<std::uint32_t>(point.timestamp_ns - start_ns);
    fused.x = static_cast<float>(transformed.x);
    fused.y = static_cast<float>(transformed.y);
    fused.z = static_cast<float>(transformed.z);
    fused.reflectivity = intensityToReflectivity(point.intensity);
    fused.tag = 0U;
    fused.line = static_cast<std::uint8_t>(4U + point.ring);
    fused.source = PointSource::kHesai;
    result.frame.points.push_back(fused);
    ++result.stats.hesai_points_output;
  }

  std::stable_sort(
    result.frame.points.begin(), result.frame.points.end(),
    [](const FusedPoint & lhs, const FusedPoint & rhs) {
      return lhs.offset_time < rhs.offset_time;
    });
  compactHesaiBuffer();
  return result;
}

void FusionCore::compactHesaiBuffer()
{
  constexpr std::size_t kCompactThreshold = 65536U;
  if (hesai_head_ == hesai_points_.size()) {
    hesai_points_.clear();
    hesai_head_ = 0U;
    return;
  }
  if (hesai_head_ >= kCompactThreshold && hesai_head_ * 2U >= hesai_points_.size()) {
    hesai_points_.erase(
      hesai_points_.begin(),
      hesai_points_.begin() + static_cast<std::ptrdiff_t>(hesai_head_));
    hesai_head_ = 0U;
  }
}

}  // namespace fastlio_go2w_fusion
