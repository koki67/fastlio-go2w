// Copyright 2026 Koki Tanaka
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef FASTLIO_GO2W_FUSION__ONLINE_WATCHDOG_HPP_
#define FASTLIO_GO2W_FUSION__ONLINE_WATCHDOG_HPP_

#include <cstdint>
#include <string>

namespace fastlio_go2w_fusion
{

enum class MissingSensorPolicy : std::uint8_t
{
  kStrict = 0U,
  kMid360Fallback = 1U,
};

enum class OnlineState : std::uint8_t
{
  kStartup = 0U,
  kHealthy = 1U,
  kStrictStopped = 2U,
  kMid360Fallback = 3U,
  kRecovering = 4U,
};

struct WatchdogUpdate
{
  OnlineState previous{OnlineState::kStartup};
  OnlineState current{OnlineState::kStartup};
  bool changed{false};
  bool reset_buffers{false};
};

class OnlineWatchdog
{
public:
  OnlineWatchdog(
    MissingSensorPolicy policy, std::uint64_t stale_timeout_ns,
    std::uint64_t startup_grace_ns, std::uint64_t start_ns);

  WatchdogUpdate observeMid(std::uint64_t now_ns);
  WatchdogUpdate observeHesai(std::uint64_t now_ns);
  WatchdogUpdate evaluate(std::uint64_t now_ns);

  OnlineState state() const;
  MissingSensorPolicy policy() const;
  bool midFresh(std::uint64_t now_ns) const;
  bool hesaiFresh(std::uint64_t now_ns) const;
  double midAgeSeconds(std::uint64_t now_ns) const;
  double hesaiAgeSeconds(std::uint64_t now_ns) const;

private:
  enum class Source : std::uint8_t {kNone = 0U, kMid = 1U, kHesai = 2U};
  WatchdogUpdate transition(std::uint64_t now_ns, Source source);
  OnlineState missingState(std::uint64_t now_ns) const;
  bool fresh(bool seen, std::uint64_t last_ns, std::uint64_t now_ns) const;
  double age(bool seen, std::uint64_t last_ns, std::uint64_t now_ns) const;

  MissingSensorPolicy policy_;
  std::uint64_t stale_timeout_ns_;
  std::uint64_t startup_grace_ns_;
  std::uint64_t start_ns_;
  OnlineState state_{OnlineState::kStartup};
  bool have_mid_{false};
  bool have_hesai_{false};
  bool recovery_mid_seen_{false};
  bool recovery_hesai_seen_{false};
  std::uint64_t last_mid_ns_{0U};
  std::uint64_t last_hesai_ns_{0U};
};

MissingSensorPolicy parseMissingSensorPolicy(const std::string & value);
const char * toString(MissingSensorPolicy policy);
const char * toString(OnlineState state);

}  // namespace fastlio_go2w_fusion

#endif  // FASTLIO_GO2W_FUSION__ONLINE_WATCHDOG_HPP_
