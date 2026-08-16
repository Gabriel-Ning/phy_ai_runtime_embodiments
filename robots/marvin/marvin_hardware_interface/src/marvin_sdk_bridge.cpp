// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
//
// Implementation of the marvin_sdk thin wrappers.
// Includes the full MarvinSDK.h header and links against libmarvin::libmarvin.

#include "marvin_hardware_interface/marvin_sdk_bridge.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <sstream>
#include <thread>

// Full SDK header — provides Robot.h / CRobot class which the C wrappers call
// into internally.  This is the only file that includes MarvinSDK.h directly.
#include "marvin/MarvinSDK.h"

namespace marvin_sdk_bridge
{

// ============================================================================
// Canonical joint limits (radians, from marvin_left_arm.xacro / marvin_right_arm.xacro)
// ============================================================================

const std::array<JointLimits, kJointsPerArm> kCanonicalJointLimits = {{
  // Joint1: ±170° (matched to ccs_m6_40.MvKDCfg)
  {-2.96706, 2.96706},
  // Joint2: lower=-2.0944, upper=2.0944 (±120°)
  {-2.0944, 2.0944},
  // Joint3: ±170°
  {-2.96706, 2.96706},
  // Joint4: lower=-2.5307, upper=1.0472
  {-2.5307, 1.0472},
  // Joint5: ±170°
  {-2.96706, 2.96706},
  // Joint6: lower=-1.0472, upper=1.0472 (±60°)
  {-1.0472, 1.0472},
  // Joint7: lower=-1.5708, upper=1.5708 (±90°)
  {-1.5708, 1.5708},
}};

// ============================================================================
// Canonical joint names
// ============================================================================

const std::unordered_set<std::string> & canonical_joint_names()
{
  static const std::unordered_set<std::string> names = {
    "Joint1_L", "Joint2_L", "Joint3_L", "Joint4_L", "Joint5_L", "Joint6_L", "Joint7_L",
    "Joint1_R", "Joint2_R", "Joint3_R", "Joint4_R", "Joint5_R", "Joint6_R", "Joint7_R",
  };
  return names;
}

const std::array<const char *, kTotalJoints> kCanonicalJointOrder = {
  "Joint1_L", "Joint2_L", "Joint3_L", "Joint4_L", "Joint5_L", "Joint6_L", "Joint7_L",
  "Joint1_R", "Joint2_R", "Joint3_R", "Joint4_R", "Joint5_R", "Joint6_R", "Joint7_R",
};

// ============================================================================
// IP validation
// ============================================================================

bool validate_ip(const std::string & ip, std::vector<int> & octets)
{
  octets.clear();
  std::istringstream iss(ip);
  std::string token;
  while (std::getline(iss, token, '.'))
  {
    if (token.empty()) return false;
    for (char c : token)
    {
      if (!std::isdigit(static_cast<unsigned char>(c))) return false;
    }
    int val;
    try { val = std::stoi(token); } catch (...) { return false; }
    if (val < 0 || val > 255) return false;
    octets.push_back(val);
  }
  return octets.size() == 4;
}

// ============================================================================
// Velocity-based per-cycle step limit
// ============================================================================

double max_step_rad(double max_joint_velocity_rad_s, double period_sec)
{
  const double p = (std::isfinite(period_sec) && period_sec > 0.0) ? period_sec : 0.0;
  const double clamped = std::clamp(p, kMinControlPeriodSec, kMaxControlPeriodSec);
  return std::min(max_joint_velocity_rad_s * clamped, kMaxPositionStepRad);
}

// ============================================================================
// Connection management
// ============================================================================

bool connect(const std::string & ip)
{
  std::vector<int> octets;
  if (!validate_ip(ip, octets)) return false;

  if (!OnLinkTo(static_cast<FX_UCHAR>(octets[0]),
                static_cast<FX_UCHAR>(octets[1]),
                static_cast<FX_UCHAR>(octets[2]),
                static_cast<FX_UCHAR>(octets[3])))
  {
    return false;
  }

  // The vendor SDK enables per-call stdout logging by default. At a 100 Hz
  // control rate this floods the console with dispatch details; hardware
  // lifecycle, safety, and error reporting remain available through ROS logs.
  OnLocalLogOff();

  // Small settle after connection
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  return true;
}

void disconnect()
{
  OnRelease();
}

long get_sdk_version()
{
  return OnGetSDKVersion();
}

bool verify_connection(int & update_count, DCSS & frame)
{
  update_count = 0;
  int prev_serial = frame.m_Out[0].m_OutFrameSerial;

  for (int i = 0; i < 5; i++)
  {
    if (!OnGetBuf(&frame))
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }
    int cur = frame.m_Out[0].m_OutFrameSerial;
    if (cur != 0 && cur != prev_serial)
    {
      update_count++;
      prev_serial = cur;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return update_count > 0;
}

// ============================================================================
// Frame I/O
// ============================================================================

bool read_frame(DCSS & frame)
{
  return OnGetBuf(&frame);
}

// ============================================================================
// State helpers
// ============================================================================

int get_arm_state(const DCSS & frame, int arm_idx)
{
  return frame.m_State[arm_idx].m_CurState;
}

int get_arm_error(const DCSS & frame, int arm_idx)
{
  return frame.m_State[arm_idx].m_ERRCode;
}

bool is_frame_fresh(const DCSS & frame, int arm_idx, int & prev_serial)
{
  int cur = frame.m_Out[arm_idx].m_OutFrameSerial;
  if (cur != prev_serial)
  {
    prev_serial = cur;
    return true;
  }
  return false;
}

// ============================================================================
// Maintenance
// ============================================================================

void read_servo_errors_left(long errors[7])
{
  OnGetServoErr_A(errors);
}

void read_servo_errors_right(long errors[7])
{
  OnGetServoErr_B(errors);
}

void clear_errors_left()
{
  OnClearErr_A();
}

void clear_errors_right()
{
  OnClearErr_B();
}

// ============================================================================
// Arm position-mode transitions
// ============================================================================

static bool wait_for_arm_state(int arm_idx, int target_state)
{
  DCSS frame{};
  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::milliseconds(kPositionStateTimeoutMs);

  while (std::chrono::steady_clock::now() < deadline)
  {
    if (!OnGetBuf(&frame))
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(kPositionStatePollIntervalMs));
      continue;
    }
    if (frame.m_State[arm_idx].m_CurState == target_state)
      return true;

    // Transient states (1xx) are normal — keep waiting.
    std::this_thread::sleep_for(std::chrono::milliseconds(kPositionStatePollIntervalMs));
  }

  return false;
}

bool enter_position_mode_left(int vel_ratio, int acc_ratio)
{
  // Combine SetJointLmt + SetTargetState into one dispatch cycle.
  // Per SDK: "切换到控制模式之前先设参数" — set params before mode switch.
  // On any staging failure, discard the transaction without dispatching
  // (OnClearSet has already cleared the buffer; a fresh cycle will start clean).
  if (!OnClearSet())  return false;
  bool ok = true;
  ok = ok && OnSetJointLmt_A(vel_ratio, acc_ratio);
  ok = ok && OnSetTargetState_A(1);
  if (!ok) return false;                // discard — do NOT call OnSetSend
  if (!OnSetSend()) return false;
  return wait_for_arm_state(0, ARM_STATE_POSITION);
}

bool enter_position_mode_right(int vel_ratio, int acc_ratio)
{
  if (!OnClearSet())  return false;
  bool ok = true;
  ok = ok && OnSetJointLmt_B(vel_ratio, acc_ratio);
  ok = ok && OnSetTargetState_B(1);
  if (!ok) return false;
  if (!OnSetSend()) return false;
  return wait_for_arm_state(1, ARM_STATE_POSITION);
}

bool exit_position_mode_left()
{
  return exit_position_modes(true, false);
}

bool exit_position_mode_right()
{
  return exit_position_modes(false, true);
}

bool exit_position_modes(bool left, bool right)
{
  if (!left && !right) return true;

  // The SDK can briefly reject a dispatch while the final position command is
  // still being consumed. Re-stage the complete transaction so a retry can
  // never send a partial bimanual state change.
  constexpr int kMaxDispatchAttempts = 3;
  bool dispatched = false;
  for (int attempt = 0; attempt < kMaxDispatchAttempts; ++attempt)
  {
    if (!OnClearSet()) return false;
    if (left && !OnSetTargetState_A(0)) return false;
    if (right && !OnSetTargetState_B(0)) return false;
    if (OnSetSend())
    {
      dispatched = true;
      break;
    }
    std::this_thread::sleep_for(
      std::chrono::milliseconds(kPositionStatePollIntervalMs));
  }
  if (!dispatched) return false;

  if (left && !wait_for_arm_state(0, ARM_STATE_IDLE)) return false;
  if (right && !wait_for_arm_state(1, ARM_STATE_IDLE)) return false;
  return true;
}

// ============================================================================
// Speed / accel limits (standalone — not used by the current hardware class,
// but available for future maintenance/preflight nodes)
// ============================================================================

bool set_joint_limits_left(int vel_ratio, int acc_ratio)
{
  if (!OnClearSet())  return false;
  if (!OnSetJointLmt_A(vel_ratio, acc_ratio)) return false;
  if (!OnSetSend()) return false;
  return true;
}

bool set_joint_limits_right(int vel_ratio, int acc_ratio)
{
  if (!OnClearSet())  return false;
  if (!OnSetJointLmt_B(vel_ratio, acc_ratio)) return false;
  if (!OnSetSend()) return false;
  return true;
}

// ============================================================================
// Combined position-command dispatch
// ============================================================================

DispatchResult dispatch_position_commands(double left_joints_deg[7], bool left_active,
                                          double right_joints_deg[7], bool right_active)
{
  if (!OnClearSet())  return DispatchResult::StagingError;
  if (left_active  && !OnSetJointCmdPos_A(left_joints_deg))  return DispatchResult::StagingError;
  if (right_active && !OnSetJointCmdPos_B(right_joints_deg)) return DispatchResult::StagingError;
  return OnSetSend() ? DispatchResult::Ok : DispatchResult::SendBusy;
}

}  // namespace marvin_sdk_bridge
