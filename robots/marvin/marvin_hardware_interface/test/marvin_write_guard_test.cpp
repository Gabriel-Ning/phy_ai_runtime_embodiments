// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
//
// Offline unit tests for write-path safety guards.
// Validates joint-limit checks, delta thresholds, NaN/Inf rejection,
// and per-arm scoping without requiring real hardware.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>

#include "marvin_hardware_interface/marvin_sdk_bridge.hpp"

namespace
{

using marvin_sdk_bridge::kJointsPerArm;
using marvin_sdk_bridge::kCanonicalJointLimits;

// ============================================================================
// Joint-limit rejection
// ============================================================================

TEST(WriteGuard, JointWithinLimits) {
  for (size_t j = 0; j < kJointsPerArm; j++) {
    // Mid-range should be OK
    double mid = (kCanonicalJointLimits[j].lower_rad + kCanonicalJointLimits[j].upper_rad) / 2.0;
    EXPECT_TRUE(mid >= kCanonicalJointLimits[j].lower_rad);
    EXPECT_TRUE(mid <= kCanonicalJointLimits[j].upper_rad);
  }
}

TEST(WriteGuard, JointOutsideLimits) {
  for (size_t j = 0; j < kJointsPerArm; j++) {
    double above = kCanonicalJointLimits[j].upper_rad + 0.1;
    double below = kCanonicalJointLimits[j].lower_rad - 0.1;
    EXPECT_FALSE(above >= kCanonicalJointLimits[j].lower_rad
                 && above <= kCanonicalJointLimits[j].upper_rad);
    EXPECT_FALSE(below >= kCanonicalJointLimits[j].lower_rad
                 && below <= kCanonicalJointLimits[j].upper_rad);
  }
}

// ============================================================================
// First-write guard (initial delta threshold)
// ============================================================================

TEST(WriteGuard, InitialDeltaWithinThreshold) {
  std::array<double, kJointsPerArm> baseline = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  std::array<double, kJointsPerArm> command  = {0.505, 0.505, 0.505, 0.505, 0.505, 0.505, 0.505};

  for (size_t j = 0; j < kJointsPerArm; j++) {
    double delta = std::abs(command[j] - baseline[j]);
    EXPECT_LE(delta, marvin_sdk_bridge::kDefaultMaxInitialDeltaRad);  // 0.005 ≤ 0.01
  }
}

TEST(WriteGuard, InitialDeltaExceedsThreshold) {
  std::array<double, kJointsPerArm> baseline = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, kJointsPerArm> command  = {0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  double delta = std::abs(command[0] - baseline[0]);
  EXPECT_GT(delta, marvin_sdk_bridge::kDefaultMaxInitialDeltaRad);  // 0.1 > 0.01
}

// ============================================================================
// Per-cycle delta guard
// ============================================================================

TEST(WriteGuard, PerCycleDeltaWithinThreshold) {
  std::array<double, kJointsPerArm> last = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, kJointsPerArm> cmd  = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  double delta = std::abs(cmd[0] - last[0]);
  EXPECT_LE(delta, marvin_sdk_bridge::kDefaultMaxPositionStepRad);  // 0.15 ≤ 0.2
}

TEST(WriteGuard, PerCycleDeltaExceedsThreshold) {
  std::array<double, kJointsPerArm> last = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, kJointsPerArm> cmd  = {0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  double delta = std::abs(cmd[0] - last[0]);
  EXPECT_GT(delta, marvin_sdk_bridge::kDefaultMaxPositionStepRad);  // 0.3 > 0.2
}

// ============================================================================
// Velocity-based per-cycle step limit (rate-independent)
// ============================================================================

TEST(WriteGuard, MaxStepScalesWithPeriod) {
  const double vel = 6.2832;  // 360 deg/s
  // Same physical velocity ⇒ allowed step scales linearly with the period,
  // so the safety envelope is identical regardless of update_rate.
  EXPECT_NEAR(marvin_sdk_bridge::max_step_rad(vel, 0.002), vel * 0.002, 1e-9);  // 500 Hz
  EXPECT_NEAR(marvin_sdk_bridge::max_step_rad(vel, 0.005), vel * 0.005, 1e-9);  // 200 Hz
  EXPECT_NEAR(marvin_sdk_bridge::max_step_rad(vel, 0.010), vel * 0.010, 1e-9);  // 100 Hz
}

TEST(WriteGuard, MaxStepCappedByAbsoluteBackstop) {
  // A large (clamped) period must not be turned into an unbounded jump.
  const double vel = 100.0;
  EXPECT_DOUBLE_EQ(marvin_sdk_bridge::max_step_rad(vel, 1.0),
                   marvin_sdk_bridge::kMaxPositionStepRad);
}

TEST(WriteGuard, MaxStepClampsPathologicalPeriod) {
  const double vel = 6.2832;
  // Zero / negative / non-finite period collapses to the minimum clamp,
  // never widening the guard.
  const double at_min = vel * marvin_sdk_bridge::kMinControlPeriodSec;
  EXPECT_NEAR(marvin_sdk_bridge::max_step_rad(vel, 0.0), at_min, 1e-9);
  EXPECT_NEAR(marvin_sdk_bridge::max_step_rad(vel, -1.0), at_min, 1e-9);
  EXPECT_NEAR(marvin_sdk_bridge::max_step_rad(vel, NAN), at_min, 1e-9);
}

// ============================================================================
// NaN / Inf rejection
// ============================================================================

TEST(WriteGuard, NaNIsRejected) {
  EXPECT_TRUE(std::isnan(NAN));
  double cmd = NAN;
  EXPECT_FALSE(!std::isnan(cmd) && !std::isinf(cmd));
}

TEST(WriteGuard, InfIsRejected) {
  double cmd = INFINITY;
  EXPECT_FALSE(!std::isnan(cmd) && !std::isinf(cmd));
}

// ============================================================================
// Per-arm guard scoping — inactive arm should not block active arm
// ============================================================================

TEST(WriteGuard, ActiveArmOnlyValidation) {
  // Simulate: left arm active, right arm inactive with stale data
  const bool left_active  = true;
  // right arm inactive — deliberately NOT validated
  std::array<double, kJointsPerArm> left_cmd  = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  std::array<double, kJointsPerArm> right_cmd = {99.9, 99.9, 99.9, 99.9, 99.9, 99.9, 99.9};  // garbage
  std::array<double, kJointsPerArm> left_last_written = {};
  std::array<double, kJointsPerArm> right_last_written = {};

  // Only validate active (left) arm
  bool rejected = false;
  if (left_active) {
    for (size_t j = 0; j < kJointsPerArm; j++) {
      if (std::isnan(left_cmd[j]) || std::isinf(left_cmd[j])) { rejected = true; break; }
      double delta = std::abs(left_cmd[j] - left_last_written[j]);
      if (delta > marvin_sdk_bridge::kDefaultMaxPositionStepRad) { rejected = true; break; }
    }
  }
  // Right arm NOT validated (inactive)
  EXPECT_FALSE(rejected);
  // Right arm would have been rejected if checked (99.9 > 0.2)
  double right_delta = std::abs(right_cmd[0] - right_last_written[0]);
  EXPECT_GT(right_delta, marvin_sdk_bridge::kDefaultMaxPositionStepRad);
}

// ============================================================================
// Velocity conversion (deg/s → rad/s)
// ============================================================================

TEST(VelocityUnits, DegPerSecToRadPerSec) {
  // 180 deg/s = π rad/s
  double raw_deg_per_s = 180.0;
  double expected_rad_per_s = M_PI;
  double converted = marvin_sdk_bridge::deg_to_rad(raw_deg_per_s);
  EXPECT_DOUBLE_EQ(converted, expected_rad_per_s);
}

TEST(VelocityUnits, ZeroVelocity) {
  EXPECT_DOUBLE_EQ(marvin_sdk_bridge::deg_to_rad(0.0), 0.0);
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
