#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace fastlio_go2w_livox
{

struct AdapterConfig
{
  double max_point_header_delta_sec{0.2};
  std::size_t minimum_points{10};
};

struct AdapterStats
{
  std::uint64_t received_frames{0};
  std::uint64_t converted_frames{0};
  std::uint64_t reordered_frames{0};
  std::uint64_t quantization_clamped_frames{0};
  std::uint64_t quantization_clamped_points{0};
  std::uint64_t dropped_invalid_header{0};
  std::uint64_t dropped_header_regression{0};
  std::uint64_t dropped_schema{0};
  std::uint64_t dropped_layout{0};
  std::uint64_t dropped_nonfinite_coordinate{0};
  std::uint64_t dropped_nonfinite_intensity{0};
  std::uint64_t dropped_nonfinite_timestamp{0};
  std::uint64_t dropped_intensity_out_of_range{0};
  std::uint64_t dropped_intensity_nonintegral{0};
  std::uint64_t dropped_negative_offset{0};
  std::uint64_t dropped_offset_too_large{0};
  std::uint64_t dropped_too_few_points{0};
  double latest_scan_width_sec{0.0};

  std::uint64_t critical_drops() const noexcept;
};

struct ConversionResult
{
  bool converted{false};
  livox_ros_driver2::msg::CustomMsg message;
  std::string drop_reason;
  bool reordered{false};
  double scan_width_sec{0.0};
};

class PointcloudAdapter
{
public:
  explicit PointcloudAdapter(AdapterConfig config = {});

  ConversionResult convert(const sensor_msgs::msg::PointCloud2 & input);

  const AdapterConfig & config() const noexcept {return config_;}
  const AdapterStats & stats() const noexcept {return stats_;}

private:
  AdapterConfig config_;
  AdapterStats stats_;
  std::optional<std::int64_t> last_header_stamp_ns_;
};

}  // namespace fastlio_go2w_livox
