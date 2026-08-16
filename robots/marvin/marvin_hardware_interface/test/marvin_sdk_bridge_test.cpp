// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
//
// Offline unit tests for marvin_sdk_bridge utilities and constants.
// No real hardware required.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "marvin_hardware_interface/marvin_sdk_bridge.hpp"

namespace
{

// ============================================================================
// deg_to_rad / rad_to_deg
// ============================================================================

TEST(SdkBridge, DegToRadZero) {
  EXPECT_DOUBLE_EQ(marvin_sdk_bridge::deg_to_rad(0.0), 0.0);
}

TEST(SdkBridge, DegToRadRoundTrip) {
  const double deg = 45.0;
  const double rad = marvin_sdk_bridge::deg_to_rad(deg);
  EXPECT_DOUBLE_EQ(marvin_sdk_bridge::rad_to_deg(rad), deg);
}

TEST(SdkBridge, DegToRadRightAngle) {
  EXPECT_DOUBLE_EQ(marvin_sdk_bridge::deg_to_rad(90.0), M_PI / 2.0);
}

// ============================================================================
// IP validation
// ============================================================================

TEST(SdkBridge, ValidateIpValid) {
  std::vector<int> octets;
  EXPECT_TRUE(marvin_sdk_bridge::validate_ip("10.19.0.191", octets));
  ASSERT_EQ(octets.size(), 4u);
  EXPECT_EQ(octets[0], 10);
  EXPECT_EQ(octets[1], 19);
  EXPECT_EQ(octets[2], 0);
  EXPECT_EQ(octets[3], 191);
}

TEST(SdkBridge, ValidateIpLocalhost) {
  std::vector<int> octets;
  EXPECT_TRUE(marvin_sdk_bridge::validate_ip("127.0.0.1", octets));
}

TEST(SdkBridge, ValidateIpRejectsBadFormat) {
  std::vector<int> octets;
  EXPECT_FALSE(marvin_sdk_bridge::validate_ip("not.an.ip", octets));
  EXPECT_FALSE(marvin_sdk_bridge::validate_ip("10.19.0", octets));
  EXPECT_FALSE(marvin_sdk_bridge::validate_ip("10.19.0.191.1", octets));
  EXPECT_FALSE(marvin_sdk_bridge::validate_ip("256.0.0.1", octets));
  EXPECT_FALSE(marvin_sdk_bridge::validate_ip("-1.0.0.1", octets));
}

// ============================================================================
// Joint limits
// ============================================================================

TEST(SdkBridge, JointLimitsCount) {
  EXPECT_EQ(marvin_sdk_bridge::kCanonicalJointLimits.size(),
            marvin_sdk_bridge::kJointsPerArm);
}

TEST(SdkBridge, JointLimitsJ1J3J5Are170Deg) {
  // J1, J3, J5 should be ±170° ≈ ±2.967 rad
  const double expected = 170.0 * M_PI / 180.0;
  EXPECT_NEAR(marvin_sdk_bridge::kCanonicalJointLimits[0].upper_rad, expected, 0.001);
  EXPECT_NEAR(marvin_sdk_bridge::kCanonicalJointLimits[0].lower_rad, -expected, 0.001);
  EXPECT_NEAR(marvin_sdk_bridge::kCanonicalJointLimits[2].upper_rad, expected, 0.001);
  EXPECT_NEAR(marvin_sdk_bridge::kCanonicalJointLimits[4].upper_rad, expected, 0.001);
}

TEST(SdkBridge, JointLimitsLowerLessThanUpper) {
  for (const auto & lim : marvin_sdk_bridge::kCanonicalJointLimits) {
    EXPECT_LT(lim.lower_rad, lim.upper_rad);
  }
}

// ============================================================================
// Joint names
// ============================================================================

TEST(SdkBridge, CanonicalJointNames) {
  const auto & names = marvin_sdk_bridge::canonical_joint_names();
  EXPECT_EQ(names.size(), 14u);
  EXPECT_NE(names.find("Joint1_L"), names.end());
  EXPECT_NE(names.find("Joint7_L"), names.end());
  EXPECT_NE(names.find("Joint1_R"), names.end());
  EXPECT_NE(names.find("Joint7_R"), names.end());
  EXPECT_EQ(names.find("Joint8_L"), names.end());
}

TEST(SdkBridge, CanonicalJointOrder) {
  const auto & order = marvin_sdk_bridge::kCanonicalJointOrder;
  EXPECT_EQ(order.size(), 14u);
  EXPECT_STREQ(order[0], "Joint1_L");
  EXPECT_STREQ(order[6], "Joint7_L");
  EXPECT_STREQ(order[7], "Joint1_R");
  EXPECT_STREQ(order[13], "Joint7_R");
}

// ============================================================================
// Constants
// ============================================================================

TEST(SdkBridge, Constants) {
  EXPECT_EQ(marvin_sdk_bridge::kJointsPerArm, 7u);
  EXPECT_EQ(marvin_sdk_bridge::kTotalJoints, 14u);
  EXPECT_GT(marvin_sdk_bridge::kDefaultMaxInitialDeltaRad, 0.0);
  EXPECT_LT(marvin_sdk_bridge::kDefaultMaxInitialDeltaRad,
            marvin_sdk_bridge::kDefaultMaxPositionStepRad);
  EXPECT_GT(marvin_sdk_bridge::kDefaultVelRatio, 0);
  EXPECT_LE(marvin_sdk_bridge::kDefaultVelRatio, 100);
}

// ============================================================================
// State helpers (compile-time / no-hardware checks)
// ============================================================================

TEST(SdkBridge, ArmStateEnum) {
  // Just verify enum values — these come from the SDK header
  EXPECT_EQ(ARM_STATE_IDLE, 0);
  EXPECT_EQ(ARM_STATE_POSITION, 1);
  EXPECT_EQ(ARM_STATE_ERROR, 100);
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
