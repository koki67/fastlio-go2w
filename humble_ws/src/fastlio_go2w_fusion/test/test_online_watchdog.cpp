// Copyright 2026 Koki Tanaka
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <cstdint>

#include "gtest/gtest.h"

#include "fastlio_go2w_fusion/online_watchdog.hpp"

namespace fusion = fastlio_go2w_fusion;

TEST(OnlineWatchdog, StrictStartupStaleAndFreshPairRecovery)
{
  fusion::OnlineWatchdog watchdog(
    fusion::MissingSensorPolicy::kStrict, 500U, 2000U, 0U);
  EXPECT_EQ(watchdog.observeMid(100U).current, fusion::OnlineState::kStartup);
  EXPECT_EQ(watchdog.observeHesai(200U).current, fusion::OnlineState::kHealthy);

  const auto stopped = watchdog.evaluate(701U);
  EXPECT_EQ(stopped.current, fusion::OnlineState::kStrictStopped);
  EXPECT_TRUE(stopped.reset_buffers);

  EXPECT_EQ(watchdog.observeHesai(800U).current, fusion::OnlineState::kStrictStopped);
  EXPECT_EQ(watchdog.observeMid(810U).current, fusion::OnlineState::kRecovering);
  EXPECT_EQ(watchdog.observeHesai(820U).current, fusion::OnlineState::kRecovering);
  EXPECT_EQ(watchdog.observeMid(830U).current, fusion::OnlineState::kHealthy);
}

TEST(OnlineWatchdog, FallbackOnlyWhenMidIsFreshAndHesaiIsStale)
{
  fusion::OnlineWatchdog watchdog(
    fusion::MissingSensorPolicy::kMid360Fallback, 500U, 2000U, 0U);
  watchdog.observeMid(100U);
  watchdog.observeHesai(100U);
  EXPECT_EQ(watchdog.evaluate(601U).current, fusion::OnlineState::kStrictStopped);

  const auto fallback = watchdog.observeMid(610U);
  EXPECT_EQ(fallback.current, fusion::OnlineState::kMid360Fallback);
  EXPECT_TRUE(fallback.reset_buffers);
  EXPECT_EQ(watchdog.evaluate(1111U).current, fusion::OnlineState::kStrictStopped);
}

TEST(OnlineWatchdog, StartupGraceDoesNotPublishFallbackEarly)
{
  fusion::OnlineWatchdog watchdog(
    fusion::MissingSensorPolicy::kMid360Fallback, 500U, 2000U, 100U);
  watchdog.observeMid(200U);
  EXPECT_EQ(watchdog.evaluate(2099U).current, fusion::OnlineState::kStartup);
  EXPECT_EQ(watchdog.evaluate(2100U).current, fusion::OnlineState::kStrictStopped);
  EXPECT_EQ(watchdog.observeMid(2110U).current, fusion::OnlineState::kMid360Fallback);
}

TEST(OnlineWatchdog, RejectsUnknownPolicy)
{
  EXPECT_EQ(
    fusion::parseMissingSensorPolicy("strict"), fusion::MissingSensorPolicy::kStrict);
  EXPECT_EQ(
    fusion::parseMissingSensorPolicy("mid360-fallback"),
    fusion::MissingSensorPolicy::kMid360Fallback);
  EXPECT_THROW(fusion::parseMissingSensorPolicy("mid-only"), std::invalid_argument);
}
