// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
//
// Thin C++ wrappers around the Marvin CCS M6 native C SDK (libMarvinSDK).
//
// This header provides types, constants, utility functions, and forward
// declarations of the SDK C API. It does NOT include the full MarvinSDK.h
// (which pulls in the CRobot C++ class) — only the minimal FxRtCSDef.h for
// frame/state structs and the ArmState enum.
//
// All SDK C functions are forward-declared here. The implementation file
// includes MarvinSDK.h directly and links against libmarvin::libmarvin.

#ifndef MARVIN_HARDWARE_INTERFACE__MARVIN_SDK_BRIDGE_HPP_
#define MARVIN_HARDWARE_INTERFACE__MARVIN_SDK_BRIDGE_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <unordered_set>
#include <vector>

// SDK data types (DCSS, StateCtr, ArmState, etc.) — minimal include.
#include "marvin/FxRtCSDef.h"

namespace marvin_sdk_bridge
{

// ============================================================================
// Constants
// ============================================================================

constexpr size_t kJointsPerArm = 7;
constexpr size_t kTotalJoints = 14;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

// Write safety
//
// Per-cycle motion is bounded by a *velocity* limit (rad/s) rather than a
// fixed per-cycle step, so the safety semantics stay constant regardless of
// the controller_manager update_rate.  The effective per-cycle step is
//   max_step = clamp(period, kMinControlPeriodSec, kMaxControlPeriodSec)
//              * max_joint_velocity
// and is additionally capped by kMaxPositionStepRad as an absolute backstop
// (protects against an abnormally large period — e.g. the first cycle after a
// stall — being turned into a huge allowed jump).
constexpr double kDefaultMaxJointVelocityRadPerSec = 6.2832;  // 360 deg/s — discontinuity guard
constexpr double kMaxPositionStepRad = 0.2;                   // absolute per-cycle backstop
constexpr double kDefaultMaxInitialDeltaRad = 0.01;

// Kept for backward compatibility / unit tests: the absolute per-cycle backstop.
constexpr double kDefaultMaxPositionStepRad = kMaxPositionStepRad;

// Period clamp for the velocity→step conversion (seconds).
constexpr double kMinControlPeriodSec = 0.0005;  // 0.5 ms (2 kHz ceiling)
constexpr double kMaxControlPeriodSec = 0.05;    // 50 ms

// Stale-frame defaults (time-based; independent of update_rate).
// A frame is "stale" when m_OutFrameSerial has not advanced.  Accumulated
// stale *time* (not cycle count) drives the warn/error thresholds.
constexpr double kDefaultStaleWarnSec = 0.02;    // 20 ms
constexpr double kDefaultStaleErrorSec = 0.10;   // 100 ms

// SDK speed/accel ratios (1–100).  10 = 10% for safety.
constexpr int kDefaultVelRatio = 100;
constexpr int kDefaultAccRatio = 100;

// Position-state transition timeout and poll interval (ms)
constexpr int kPositionStateTimeoutMs = 3000;
constexpr int kPositionStatePollIntervalMs = 50;

// ============================================================================
// Canonical joint limits (radians, from URDF; symmetric left / right)
// ============================================================================

struct JointLimits
{
  double lower_rad;
  double upper_rad;
};

/// Per-arm limits for Joint1–Joint7 in the canonical Marvin joint order.
/// Left and right arms share the same limits.
extern const std::array<JointLimits, kJointsPerArm> kCanonicalJointLimits;

// ============================================================================
// Canonical joint names
// ============================================================================

const std::unordered_set<std::string> & canonical_joint_names();

/// Ordered canonical joint list: Joint1_L..Joint7_L, Joint1_R..Joint7_R.
/// Used to fix the export order regardless of URDF declaration order.
extern const std::array<const char *, kTotalJoints> kCanonicalJointOrder;

// ============================================================================
// Utilities
// ============================================================================

bool validate_ip(const std::string & ip, std::vector<int> & octets);

inline double deg_to_rad(double deg) { return deg * kDegToRad; }
inline double rad_to_deg(double rad) { return rad / kDegToRad; }

/// Velocity-based per-cycle step limit (rad), independent of update_rate.
/// Returns clamp(period, kMin, kMax) * max_joint_velocity, capped by the
/// absolute backstop kMaxPositionStepRad.  A non-finite/zero period collapses
/// to the minimum clamp so the guard never widens unexpectedly.
double max_step_rad(double max_joint_velocity_rad_s, double period_sec);

// ============================================================================
// SDK C function declarations
// ============================================================================
// These are the raw C-linkage functions exported by libMarvinSDK.
// Declared here so client code does not need to include <marvin/MarvinSDK.h>.

extern "C" {

// Connection
bool OnLinkTo(unsigned char ip1, unsigned char ip2, unsigned char ip3, unsigned char ip4);
bool OnRelease();
long OnGetSDKVersion();

// Frame I/O
bool OnGetBuf(DCSS * ret);

// Maintenance (read-only)
void OnGetServoErr_A(long ErrCode[7]);
void OnGetServoErr_B(long ErrCode[7]);
void OnClearErr_A();
void OnClearErr_B();

// Dispatch envelope — all commands must be wrapped:
//   OnClearSet() → <commands> → OnSetSend()
bool OnClearSet();
bool OnSetSend();

// Position commands (degrees)
bool OnSetJointCmdPos_A(double joint[7]);
bool OnSetJointCmdPos_B(double joint[7]);

// Target state (0=idle, 1=position, 2=PVT, 3=torque, 4=release)
bool OnSetTargetState_A(int state);
bool OnSetTargetState_B(int state);

// Speed / acceleration limits (1–100)
bool OnSetJointLmt_A(int velRatio, int AccRatio);
bool OnSetJointLmt_B(int velRatio, int AccRatio);

}  // extern "C"

// ============================================================================
// Combined position-command dispatch
// ============================================================================
// Encapsulates the full OnClearSet → commands → OnSetSend transaction.

enum class DispatchResult { Ok, SendBusy, StagingError };

/// Dispatch position commands to the active arm(s).
/// - Ok:           all staging + send succeeded.
/// - SendBusy:     staging ok, but OnSetSend tag was still set (transient race).
/// - StagingError: OnClearSet or OnSetJointCmdPos failed (real error).
/// Staging failures discard the transaction without partial dispatch.
DispatchResult dispatch_position_commands(double left_joints_deg[7], bool left_active,
                                          double right_joints_deg[7], bool right_active);

// ============================================================================
// Connection management
// ============================================================================

/// Connect to the controller at the given dotted-quad IPv4 address.
/// Returns true on success.
bool connect(const std::string & ip);

/// Release the SDK connection.
void disconnect();

/// Return the SDK build version (informational).
long get_sdk_version();

/// Verify the connection is alive by checking for fresh frame updates.
/// Returns true and sets `update_count` if at least one new frame arrived.
bool verify_connection(int & update_count, DCSS & frame);

// ============================================================================
// Frame I/O
// ============================================================================

/// Read one data frame from the controller.
/// Returns true on success.
bool read_frame(DCSS & frame);

// ============================================================================
// State helpers
// ============================================================================

/// Return m_CurState for the given arm (0=left, 1=right).
int get_arm_state(const DCSS & frame, int arm_idx);

/// Return m_ERRCode for the given arm (0=left, 1=right).
int get_arm_error(const DCSS & frame, int arm_idx);

/// Check whether the frame for `arm_idx` is newer than `prev_serial`.
/// Updates `prev_serial` when a new frame arrives.
bool is_frame_fresh(const DCSS & frame, int arm_idx, int & prev_serial);

// ============================================================================
// Maintenance (these call into the SDK directly — use with caution)
// ============================================================================

void read_servo_errors_left(long errors[7]);
void read_servo_errors_right(long errors[7]);
void clear_errors_left();
void clear_errors_right();

// ============================================================================
// Arm position-mode transitions
// ============================================================================
// Each call wraps OnClearSet → OnSetJointLmt + OnSetTargetState → OnSetSend
// in a single dispatch cycle, then polls OnGetBuf until the arm reaches the
// target state or kPositionStateTimeoutMs.

/// Transition left arm to position-following mode (state 1).
/// Sets speed/accel limits and target state in one SDK dispatch cycle.
/// Blocks for up to kPositionStateTimeoutMs.
/// Returns true if the arm reached ARM_STATE_POSITION.
bool enter_position_mode_left(int vel_ratio, int acc_ratio);

/// Transition right arm to position-following mode (state 1).
bool enter_position_mode_right(int vel_ratio, int acc_ratio);

/// Transition left arm to idle (state 0).
bool exit_position_mode_left();

/// Transition right arm to idle (state 0).
bool exit_position_mode_right();

/// Transition the selected arms to idle in one SDK dispatch transaction.
/// Returns true only after every selected arm reports ARM_STATE_IDLE.
bool exit_position_modes(bool left, bool right);

}  // namespace marvin_sdk_bridge

#endif  // MARVIN_HARDWARE_INTERFACE__MARVIN_SDK_BRIDGE_HPP_
