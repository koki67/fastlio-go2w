// Copyright 2026 Koki Tanaka
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "fastlio_go2w_fusion/online_watchdog.hpp"

#include <limits>
#include <stdexcept>

namespace fastlio_go2w_fusion
{

OnlineWatchdog::OnlineWatchdog(
  MissingSensorPolicy policy, std::uint64_t stale_timeout_ns,
  std::uint64_t startup_grace_ns, std::uint64_t start_ns)
: policy_(policy),
  stale_timeout_ns_(stale_timeout_ns),
  startup_grace_ns_(startup_grace_ns),
  start_ns_(start_ns)
{
  if (stale_timeout_ns_ == 0U) {
    throw std::invalid_argument("source stale timeout must be positive");
  }
}

WatchdogUpdate OnlineWatchdog::observeMid(std::uint64_t now_ns)
{
  have_mid_ = true;
  last_mid_ns_ = now_ns;
  return transition(now_ns, Source::kMid);
}

WatchdogUpdate OnlineWatchdog::observeHesai(std::uint64_t now_ns)
{
  have_hesai_ = true;
  last_hesai_ns_ = now_ns;
  return transition(now_ns, Source::kHesai);
}

WatchdogUpdate OnlineWatchdog::evaluate(std::uint64_t now_ns)
{
  return transition(now_ns, Source::kNone);
}

OnlineState OnlineWatchdog::state() const
{
  return state_;
}

MissingSensorPolicy OnlineWatchdog::policy() const
{
  return policy_;
}

bool OnlineWatchdog::fresh(
  bool seen, std::uint64_t last_ns, std::uint64_t now_ns) const
{
  return seen && now_ns >= last_ns && now_ns - last_ns <= stale_timeout_ns_;
}

bool OnlineWatchdog::midFresh(std::uint64_t now_ns) const
{
  return fresh(have_mid_, last_mid_ns_, now_ns);
}

bool OnlineWatchdog::hesaiFresh(std::uint64_t now_ns) const
{
  return fresh(have_hesai_, last_hesai_ns_, now_ns);
}

double OnlineWatchdog::age(
  bool seen, std::uint64_t last_ns, std::uint64_t now_ns) const
{
  if (!seen || now_ns < last_ns) {
    return std::numeric_limits<double>::infinity();
  }
  return static_cast<double>(now_ns - last_ns) / 1.0e9;
}

double OnlineWatchdog::midAgeSeconds(std::uint64_t now_ns) const
{
  return age(have_mid_, last_mid_ns_, now_ns);
}

double OnlineWatchdog::hesaiAgeSeconds(std::uint64_t now_ns) const
{
  return age(have_hesai_, last_hesai_ns_, now_ns);
}

OnlineState OnlineWatchdog::missingState(std::uint64_t now_ns) const
{
  if (policy_ == MissingSensorPolicy::kMid360Fallback &&
    midFresh(now_ns) && !hesaiFresh(now_ns))
  {
    return OnlineState::kMid360Fallback;
  }
  return OnlineState::kStrictStopped;
}

WatchdogUpdate OnlineWatchdog::transition(std::uint64_t now_ns, Source source)
{
  WatchdogUpdate update;
  update.previous = state_;

  if (state_ == OnlineState::kRecovering) {
    if (source == Source::kMid) {
      recovery_mid_seen_ = true;
    } else if (source == Source::kHesai) {
      recovery_hesai_seen_ = true;
    }
  }

  const bool both_fresh = midFresh(now_ns) && hesaiFresh(now_ns);
  switch (state_) {
    case OnlineState::kStartup:
      if (both_fresh) {
        state_ = OnlineState::kHealthy;
      } else if (now_ns >= start_ns_ && now_ns - start_ns_ >= startup_grace_ns_) {
        state_ = missingState(now_ns);
        update.reset_buffers = true;
      }
      break;
    case OnlineState::kHealthy:
      if (!both_fresh) {
        state_ = missingState(now_ns);
        update.reset_buffers = true;
      }
      break;
    case OnlineState::kStrictStopped:
    case OnlineState::kMid360Fallback:
      if (both_fresh) {
        state_ = OnlineState::kRecovering;
        recovery_mid_seen_ = false;
        recovery_hesai_seen_ = false;
        update.reset_buffers = true;
      } else {
        const OnlineState missing = missingState(now_ns);
        if (missing != state_) {
          state_ = missing;
          update.reset_buffers = true;
        }
      }
      break;
    case OnlineState::kRecovering:
      if (!both_fresh) {
        state_ = missingState(now_ns);
        recovery_mid_seen_ = false;
        recovery_hesai_seen_ = false;
        update.reset_buffers = true;
      } else if (recovery_mid_seen_ && recovery_hesai_seen_) {
        state_ = OnlineState::kHealthy;
      }
      break;
  }

  update.current = state_;
  update.changed = update.current != update.previous;
  return update;
}

MissingSensorPolicy parseMissingSensorPolicy(const std::string & value)
{
  if (value == "strict") {
    return MissingSensorPolicy::kStrict;
  }
  if (value == "mid360-fallback") {
    return MissingSensorPolicy::kMid360Fallback;
  }
  throw std::invalid_argument(
          "missing_sensor_policy must be strict or mid360-fallback");
}

const char * toString(MissingSensorPolicy policy)
{
  return policy == MissingSensorPolicy::kStrict ? "strict" : "mid360-fallback";
}

const char * toString(OnlineState state)
{
  switch (state) {
    case OnlineState::kStartup: return "startup";
    case OnlineState::kHealthy: return "healthy";
    case OnlineState::kStrictStopped: return "strict_stopped";
    case OnlineState::kMid360Fallback: return "mid360_fallback";
    case OnlineState::kRecovering: return "recovering";
  }
  return "unknown";
}

}  // namespace fastlio_go2w_fusion
