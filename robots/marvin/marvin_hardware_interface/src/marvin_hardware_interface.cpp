// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
//
// Marvin CCS M6 bimanual arm hardware interface implementation.
// See the header for architecture overview.
//
// SDK details live in marvin_sdk_bridge.hpp / marvin_sdk_bridge.cpp.
// This file owns ros2_control lifecycle, mode switching, and RT read / write.

#include "marvin_hardware_interface/marvin_hardware_interface.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace marvin_hardware_interface
{

using marvin_sdk_bridge::kJointsPerArm;
using marvin_sdk_bridge::kTotalJoints;

// ============================================================================
// Helper: parse a key from hardware_parameters unordered_map
// ============================================================================

static std::string get_hw_param(
    const std::unordered_map<std::string, std::string> & hp,
    const std::string & key, const std::string & default_val)
{
  auto it = hp.find(key);
  return (it != hp.end()) ? it->second : default_val;
}

// ============================================================================
// Lifecycle: on_init
// ============================================================================

hardware_interface::CallbackReturn MarvinBimanualArmHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  const auto & info = params.hardware_info;

  RCLCPP_INFO(logger_, "MarvinBimanualArmHardware on_init — %zu joints in URDF",
              info.joints.size());

  if (info.joints.size() != kTotalJoints)
  {
    RCLCPP_ERROR(logger_, "Expected %zu joints, got %zu.", kTotalJoints, info.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Validate against the canonical name set (order-independent).
  {
    const auto & canonical = marvin_sdk_bridge::canonical_joint_names();
    for (const auto & j : info.joints)
    {
      if (canonical.find(j.name) == canonical.end())
      {
        RCLCPP_ERROR(logger_, "Unknown joint '%s' — expected canonical Marvin 14 joints.", j.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  // Use the fixed canonical order, not the URDF declaration order.
  joint_names_.clear();
  for (const auto * name : marvin_sdk_bridge::kCanonicalJointOrder)
    joint_names_.emplace_back(name);

  const auto & hp = info.hardware_parameters;
  robot_ip_ = get_hw_param(hp, "robot_ip", "10.19.0.191");
  use_fake_hardware_ = (get_hw_param(hp, "use_fake_hardware", "true") != "false");

  // Time-based safety thresholds (configurable from the URDF), so their
  // physical meaning does not change when the controller_manager update_rate
  // changes.  Parsing is defensive: a malformed value falls back to default.
  auto parse_double = [&](const char * key, double fallback) {
    auto it = hp.find(key);
    if (it == hp.end()) return fallback;
    try {
      size_t pos = 0;
      double v = std::stod(it->second, &pos);
      // Reject trailing garbage (e.g. "6.28rad"): only trailing whitespace is OK.
      while (pos < it->second.size() &&
             std::isspace(static_cast<unsigned char>(it->second[pos]))) ++pos;
      if (pos != it->second.size()) throw std::invalid_argument(it->second);
      return v;
    } catch (...) {
      RCLCPP_WARN(logger_, "Invalid %s='%s' — using %.4f", key, it->second.c_str(), fallback);
      return fallback;
    }
  };
  stale_warn_sec_  = parse_double("stale_warn_ms",  marvin_sdk_bridge::kDefaultStaleWarnSec  * 1e3) / 1e3;
  stale_error_sec_ = parse_double("stale_error_ms", marvin_sdk_bridge::kDefaultStaleErrorSec * 1e3) / 1e3;
  max_joint_velocity_ = parse_double("max_joint_velocity",
                                     marvin_sdk_bridge::kDefaultMaxJointVelocityRadPerSec);
  if (stale_warn_sec_ <= 0.0)  stale_warn_sec_  = marvin_sdk_bridge::kDefaultStaleWarnSec;
  if (stale_error_sec_ <= 0.0) stale_error_sec_ = marvin_sdk_bridge::kDefaultStaleErrorSec;
  if (max_joint_velocity_ <= 0.0)
    max_joint_velocity_ = marvin_sdk_bridge::kDefaultMaxJointVelocityRadPerSec;

  RCLCPP_INFO(logger_, "robot_ip=%s fake=%s stale_warn=%.0fms stale_error=%.0fms max_vel=%.3frad/s",
              robot_ip_.c_str(), use_fake_hardware_ ? "true" : "false",
              stale_warn_sec_ * 1e3, stale_error_sec_ * 1e3, max_joint_velocity_);

  // Allocate buffers
  hw_position_states_.assign(kTotalJoints, 0.0);
  hw_velocity_states_.assign(kTotalJoints, 0.0);
  hw_effort_states_.assign(kTotalJoints, 0.0);
  hw_position_commands_.assign(kTotalJoints, 0.0);
  hw_commands_deg_buffer_.assign(kTotalJoints, 0.0);

  // Zero per-arm safety state
  left_activation_baseline_.fill(0.0);
  right_activation_baseline_.fill(0.0);
  left_last_written_.fill(0.0);
  right_last_written_.fill(0.0);

  RCLCPP_INFO(logger_, "on_init OK — %zu joints ready", kTotalJoints);
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// Lifecycle: on_configure
// ============================================================================

hardware_interface::CallbackReturn MarvinBimanualArmHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "on_configure — connecting to %s", robot_ip_.c_str());

  if (use_fake_hardware_)
  {
    RCLCPP_INFO(logger_, "Fake hardware mode — skipping SDK connection");
    hardware_connected_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Validate IP format
  std::vector<int> octets;
  if (!marvin_sdk_bridge::validate_ip(robot_ip_, octets))
  {
    RCLCPP_ERROR(logger_, "Invalid robot_ip '%s'", robot_ip_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Connect via SDK
  if (!marvin_sdk_bridge::connect(robot_ip_))
  {
    RCLCPP_ERROR(logger_, "marvin_sdk_bridge::connect failed for %s", robot_ip_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "SDK connection established — version %ld",
              marvin_sdk_bridge::get_sdk_version());

  // Read initial frame for informational diagnostics (no control ops)
  {
    marvin_sdk_bridge::read_frame(frame_data_);
    RCLCPP_INFO(logger_, "Raw arm states — Left: %d, Right: %d",
                marvin_sdk_bridge::get_arm_state(frame_data_, 0),
                marvin_sdk_bridge::get_arm_state(frame_data_, 1));

    long left_errs[7] = {}, right_errs[7] = {};
    marvin_sdk_bridge::read_servo_errors_left(left_errs);
    marvin_sdk_bridge::read_servo_errors_right(right_errs);

    bool has_err = false;
    for (int i = 0; i < 7; i++)
    {
      if (left_errs[i]  != 0) { RCLCPP_WARN(logger_, "Left  J%d servo err: %ld", i+1, left_errs[i]);  has_err = true; }
      if (right_errs[i] != 0) { RCLCPP_WARN(logger_, "Right J%d servo err: %ld", i+1, right_errs[i]); has_err = true; }
    }
    if (!has_err) RCLCPP_INFO(logger_, "Raw servo errors — none detected");
  }

  // Verify connection with frame serial checks
  int update_count = 0;
  if (!marvin_sdk_bridge::verify_connection(update_count, frame_data_))
  {
    RCLCPP_ERROR(logger_, "No frame updates — connection verification failed");
    marvin_sdk_bridge::disconnect();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "Connection verified — %d frame updates", update_count);
  hardware_connected_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// Lifecycle: on_activate
// ============================================================================

hardware_interface::CallbackReturn MarvinBimanualArmHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "on_activate");

  if (!hardware_connected_)
  {
    RCLCPP_ERROR(logger_, "Not connected — cannot activate");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (use_fake_hardware_)
  {
    RCLCPP_INFO(logger_, "Fake hardware — activate OK");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Drain initial frames and seed state
  RCLCPP_INFO(logger_, "Draining initial SDK frames...");
  for (int i = 0; i < 5; i++)
  {
    marvin_sdk_bridge::read_frame(frame_data_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  prev_left_serial_  = frame_data_.m_Out[0].m_OutFrameSerial;
  prev_right_serial_ = frame_data_.m_Out[1].m_OutFrameSerial;
  left_stale_sec_  = 0.0;
  right_stale_sec_ = 0.0;
  dispatch_skip_sec_     = 0.0;
  dispatch_skipped_prev_ = false;

  // Seeding read — do not advance the stale clock (period 0).
  if (!read_frame_from_hardware(0.0))
    RCLCPP_WARN(logger_, "Initial read returned no valid data");

  // Seed command buffers and per-arm safety state from current measured state.
  // Activation baselines are set here; they may be overwritten later in
  // perform_command_mode_switch when a specific arm becomes active.
  for (size_t i = 0; i < kTotalJoints; i++)
    hw_position_commands_[i] = hw_position_states_[i];

  for (size_t j = 0; j < kJointsPerArm; j++)
  {
    left_activation_baseline_[j]  = hw_position_states_[j];
    right_activation_baseline_[j] = hw_position_states_[kJointsPerArm + j];
    left_last_written_[j]  = hw_position_states_[j];
    right_last_written_[j] = hw_position_states_[kJointsPerArm + j];
  }

  RCLCPP_INFO(logger_, "on_activate OK — states seeded, write guards armed");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// Lifecycle: on_deactivate
// ============================================================================

hardware_interface::CallbackReturn MarvinBimanualArmHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "on_deactivate");

  // Snapshot and clear active-arm flags so write() stops dispatching.
  const bool left_was_active  = left_position_active_.exchange(false);
  const bool right_was_active = right_position_active_.exchange(false);
  left_initial_write_pending_ = false;
  right_initial_write_pending_ = false;

  bool exit_ok = true;
  if (!use_fake_hardware_ && hardware_connected_)
  {
    if (left_was_active || right_was_active)
    {
      RCLCPP_INFO(
        logger_, "on_deactivate — exiting position mode (left=%s right=%s)",
        left_was_active ? "yes" : "no", right_was_active ? "yes" : "no");
      exit_ok = marvin_sdk_bridge::exit_position_modes(left_was_active, right_was_active);
      if (!exit_ok)
        RCLCPP_ERROR(logger_, "on_deactivate — exit-position-mode state confirmation failed");
    }

    marvin_sdk_bridge::disconnect();
  }
  hardware_connected_ = false;

  return exit_ok ? hardware_interface::CallbackReturn::SUCCESS
                 : hardware_interface::CallbackReturn::ERROR;
}

// ============================================================================
// Lifecycle: on_cleanup / on_error
// ============================================================================

hardware_interface::CallbackReturn MarvinBimanualArmHardware::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "on_cleanup");

  const bool left_was_active  = left_position_active_.exchange(false);
  const bool right_was_active = right_position_active_.exchange(false);
  left_initial_write_pending_ = false;
  right_initial_write_pending_ = false;

  if (!use_fake_hardware_ && hardware_connected_)
  {
    bool exit_ok = true;
    if (left_was_active || right_was_active)
    {
      RCLCPP_INFO(
        logger_, "on_cleanup — exiting position mode (left=%s right=%s)",
        left_was_active ? "yes" : "no", right_was_active ? "yes" : "no");
      exit_ok = marvin_sdk_bridge::exit_position_modes(left_was_active, right_was_active);
    }
    marvin_sdk_bridge::disconnect();
    hardware_connected_ = false;
    if (!exit_ok)
    {
      RCLCPP_ERROR(logger_, "on_cleanup — exit-position-mode state confirmation failed");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  hardware_connected_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MarvinBimanualArmHardware::on_error(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(logger_, "on_error — attempting safe shutdown");

  const bool left_was_active  = left_position_active_.exchange(false);
  const bool right_was_active = right_position_active_.exchange(false);
  left_initial_write_pending_ = false;
  right_initial_write_pending_ = false;

  if (!use_fake_hardware_ && hardware_connected_)
  {
    bool exit_ok = true;
    if (left_was_active || right_was_active)
    {
      RCLCPP_ERROR(
        logger_, "on_error — exiting position mode (left=%s right=%s)",
        left_was_active ? "yes" : "no", right_was_active ? "yes" : "no");
      exit_ok = marvin_sdk_bridge::exit_position_modes(left_was_active, right_was_active);
    }
    marvin_sdk_bridge::disconnect();
    hardware_connected_ = false;
    if (!exit_ok)
    {
      RCLCPP_ERROR(logger_, "on_error — exit-position-mode state confirmation failed");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  hardware_connected_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// State Interface Export
// ============================================================================

std::vector<hardware_interface::StateInterface>
MarvinBimanualArmHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(kTotalJoints * 3);

  for (size_t i = 0; i < kTotalJoints; i++)
  {
    const auto & name = joint_names_[i];
    state_interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]);
    state_interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[i]);
    state_interfaces.emplace_back(name, hardware_interface::HW_IF_EFFORT,   &hw_effort_states_[i]);
  }
  return state_interfaces;
}

// ============================================================================
// Command Interface Export
// ============================================================================

std::vector<hardware_interface::CommandInterface>
MarvinBimanualArmHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(kTotalJoints);

  for (size_t i = 0; i < kTotalJoints; i++)
    command_interfaces.emplace_back(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]);

  return command_interfaces;
}

// ============================================================================
// Real-time read()
// ============================================================================

hardware_interface::return_type MarvinBimanualArmHardware::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!hardware_connected_ || use_fake_hardware_)
    return hardware_interface::return_type::OK;

  if (!read_frame_from_hardware(period.seconds()))
    return hardware_interface::return_type::ERROR;

  return hardware_interface::return_type::OK;
}

// ============================================================================
// Real-time write() — per-arm dispatch with safety guards
// ============================================================================

hardware_interface::return_type MarvinBimanualArmHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (use_fake_hardware_)
    return hardware_interface::return_type::OK;

  const bool left_active  = left_position_active_;
  const bool right_active = right_position_active_;

  if (!left_active && !right_active)
  {
    RCLCPP_INFO_THROTTLE(logger_, *get_node()->get_clock(), 30000,
                         "write() no-op — no arm active");
    return hardware_interface::return_type::OK;
  }

  // --- Velocity-based per-cycle step limit (time-correct, rate-independent) ---
  const double max_step = marvin_sdk_bridge::max_step_rad(max_joint_velocity_, period.seconds());

  // --- Per-arm stale frame guard (time-based) ---
  if (left_active && left_stale_sec_ >= stale_error_sec_)
  {
    RCLCPP_ERROR_THROTTLE(logger_, *get_node()->get_clock(), 2000,
                          "write() rejected — left arm stale %.0f ms", left_stale_sec_ * 1e3);
    return hardware_interface::return_type::ERROR;
  }
  if (right_active && right_stale_sec_ >= stale_error_sec_)
  {
    RCLCPP_ERROR_THROTTLE(logger_, *get_node()->get_clock(), 2000,
                          "write() rejected — right arm stale %.0f ms", right_stale_sec_ * 1e3);
    return hardware_interface::return_type::ERROR;
  }

  // --- Arm error guard (check active arms) ---
  if (left_active && marvin_sdk_bridge::get_arm_error(frame_data_, 0) != 0)
  {
    RCLCPP_ERROR_THROTTLE(logger_, *get_node()->get_clock(), 2000,
                          "write() rejected — left arm error %d",
                          marvin_sdk_bridge::get_arm_error(frame_data_, 0));
    return hardware_interface::return_type::ERROR;
  }
  if (right_active && marvin_sdk_bridge::get_arm_error(frame_data_, 1) != 0)
  {
    RCLCPP_ERROR_THROTTLE(logger_, *get_node()->get_clock(), 2000,
                          "write() rejected — right arm error %d",
                          marvin_sdk_bridge::get_arm_error(frame_data_, 1));
    return hardware_interface::return_type::ERROR;
  }

  // --- Convert rad to deg for all joints (needed for both arms) ---
  for (size_t i = 0; i < kTotalJoints; i++)
    hw_commands_deg_buffer_[i] = marvin_sdk_bridge::rad_to_deg(hw_position_commands_[i]);

  // --- Re-baseline after a skip (T3) ---
  // Must run BEFORE the per-arm guards so they compare against the
  // freshly re-baselined last_written_/activation_baseline_ (= measured
  // state), not the stale pre-skip values that JTC drifted away from.
  if (dispatch_skipped_prev_)
  {
    if (left_active)
    {
      for (size_t j = 0; j < kJointsPerArm; j++)
        left_last_written_[j] = hw_position_states_[j];
      if (left_initial_write_pending_)
        for (size_t j = 0; j < kJointsPerArm; j++)
          left_activation_baseline_[j] = hw_position_states_[j];
    }
    if (right_active)
    {
      for (size_t j = 0; j < kJointsPerArm; j++)
        right_last_written_[j] = hw_position_states_[kJointsPerArm + j];
      if (right_initial_write_pending_)
        for (size_t j = 0; j < kJointsPerArm; j++)
          right_activation_baseline_[j] = hw_position_states_[kJointsPerArm + j];
    }
    dispatch_skipped_prev_ = false;
  }

  // --- Per-arm validation and dispatch ---

  // Validate and dispatch left arm
  if (left_active)
  {
    // Joint limit check (Fix #4)
    for (size_t j = 0; j < kJointsPerArm; j++)
    {
      const auto & limits = marvin_sdk_bridge::kCanonicalJointLimits[j];
      double cmd = hw_position_commands_[j];
      if (cmd < limits.lower_rad || cmd > limits.upper_rad)
      {
        RCLCPP_ERROR_THROTTLE(logger_, *get_node()->get_clock(), 2000,
                              "write() rejected — left J%zu cmd %.4f outside [%.4f, %.4f]",
                              j + 1, cmd, limits.lower_rad, limits.upper_rad);
        return hardware_interface::return_type::ERROR;
      }
    }

    // NaN/Inf guard (Fix #2: active arm only)
    for (size_t j = 0; j < kJointsPerArm; j++)
    {
      if (std::isnan(hw_position_commands_[j]) || std::isinf(hw_position_commands_[j]))
      {
        RCLCPP_ERROR_THROTTLE(logger_, *get_node()->get_clock(), 2000,
                              "write() rejected — left J%zu NaN/Inf", j + 1);
        return hardware_interface::return_type::ERROR;
      }
    }

    // First-write guard (Fix #1): compare against activation baseline
    if (left_initial_write_pending_)
    {
      for (size_t j = 0; j < kJointsPerArm; j++)
      {
        double delta = std::abs(hw_position_commands_[j] - left_activation_baseline_[j]);
        if (delta > marvin_sdk_bridge::kDefaultMaxInitialDeltaRad)
        {
          RCLCPP_ERROR_THROTTLE(logger_, *get_node()->get_clock(), 2000,
                                "write() rejected — left J%zu initial delta %.4f > %.4f rad "
                                "(cmd=%.4f, baseline=%.4f)",
                                j + 1, delta, marvin_sdk_bridge::kDefaultMaxInitialDeltaRad,
                                hw_position_commands_[j], left_activation_baseline_[j]);
          return hardware_interface::return_type::ERROR;
        }
      }
    }
    else
    {
      // Per-cycle delta guard (active arm only) — velocity × period.
      for (size_t j = 0; j < kJointsPerArm; j++)
      {
        double delta = std::abs(hw_position_commands_[j] - left_last_written_[j]);
        if (delta > max_step)
        {
          RCLCPP_ERROR_THROTTLE(logger_, *get_node()->get_clock(), 2000,
                                "write() rejected — left J%zu step %.4f > %.4f rad "
                                "(limit %.2f rad/s)",
                                j + 1, delta, max_step, max_joint_velocity_);
          return hardware_interface::return_type::ERROR;
        }
      }
    }
  }

  // Right arm — same pattern
  if (right_active)
  {
    for (size_t j = 0; j < kJointsPerArm; j++)
    {
      const auto & limits = marvin_sdk_bridge::kCanonicalJointLimits[j];
      double cmd = hw_position_commands_[kJointsPerArm + j];
      if (cmd < limits.lower_rad || cmd > limits.upper_rad)
      {
        RCLCPP_ERROR_THROTTLE(logger_, *get_node()->get_clock(), 2000,
                              "write() rejected — right J%zu cmd %.4f outside [%.4f, %.4f]",
                              j + 1, cmd, limits.lower_rad, limits.upper_rad);
        return hardware_interface::return_type::ERROR;
      }
    }

    for (size_t j = 0; j < kJointsPerArm; j++)
    {
      size_t idx = kJointsPerArm + j;
      if (std::isnan(hw_position_commands_[idx]) || std::isinf(hw_position_commands_[idx]))
      {
        RCLCPP_ERROR_THROTTLE(logger_, *get_node()->get_clock(), 2000,
                              "write() rejected — right J%zu NaN/Inf", j + 1);
        return hardware_interface::return_type::ERROR;
      }
    }

    if (right_initial_write_pending_)
    {
      for (size_t j = 0; j < kJointsPerArm; j++)
      {
        double cmd = hw_position_commands_[kJointsPerArm + j];
        double delta = std::abs(cmd - right_activation_baseline_[j]);
        if (delta > marvin_sdk_bridge::kDefaultMaxInitialDeltaRad)
        {
          RCLCPP_ERROR_THROTTLE(logger_, *get_node()->get_clock(), 2000,
                                "write() rejected — right J%zu initial delta %.4f > %.4f rad",
                                j + 1, delta, marvin_sdk_bridge::kDefaultMaxInitialDeltaRad);
          return hardware_interface::return_type::ERROR;
        }
      }
    }
    else
    {
      for (size_t j = 0; j < kJointsPerArm; j++)
      {
        double cmd = hw_position_commands_[kJointsPerArm + j];
        double delta = std::abs(cmd - right_last_written_[j]);
        if (delta > max_step)
        {
          RCLCPP_ERROR_THROTTLE(logger_, *get_node()->get_clock(), 2000,
                                "write() rejected — right J%zu step %.4f > %.4f rad "
                                "(limit %.2f rad/s)",
                                j + 1, delta, max_step, max_joint_velocity_);
          return hardware_interface::return_type::ERROR;
        }
      }
    }
  }

  // --- SDK dispatch (tri-state: T1) ---
  const auto result = marvin_sdk_bridge::dispatch_position_commands(
      hw_commands_deg_buffer_.data(), left_active,
      hw_commands_deg_buffer_.data() + kJointsPerArm, right_active);

  using marvin_sdk_bridge::DispatchResult;
  if (result != DispatchResult::Ok)
  {
    // StagingError (e.g. cold-start transient after mode switch) and
    // SendBusy (timer-thread race) are both recoverable.  Accumulate and
    // only escalate to ERROR if sustained beyond stale_error_sec_.
    //
    // Clamp the per-cycle contribution to kMaxControlPeriodSec (50 ms).
    // Without this clamp, a single controller_manager overrun (e.g. the
    // ~600 ms perform_command_mode_switch → enter_position_mode_* blocking
    // call) injects the full overrun duration into the accumulator,
    // instantly exceeding the 100 ms error threshold on the very first
    // post-overrun write cycle — a false positive.
    const double clamped_dt = std::min(
        period.seconds(), marvin_sdk_bridge::kMaxControlPeriodSec);
    dispatch_skip_sec_ += clamped_dt;
    dispatch_skipped_prev_ = true;

    if (dispatch_skip_sec_ >= stale_error_sec_)
    {
      RCLCPP_ERROR(logger_, "write() — dispatch %s sustained %.0f ms >= error %.0f ms",
                   result == DispatchResult::StagingError ? "staging error" : "tag busy",
                   dispatch_skip_sec_ * 1e3, stale_error_sec_ * 1e3);
      dispatch_skip_sec_ = 0.0;
      return hardware_interface::return_type::ERROR;
    }
    RCLCPP_WARN_THROTTLE(logger_, *get_node()->get_clock(), 2000,
                         "write() — dispatch %s (%.0f ms accumulated)",
                         result == DispatchResult::StagingError ? "staging error" : "tag busy",
                         dispatch_skip_sec_ * 1e3);
    return hardware_interface::return_type::OK;
  }

  // Ok — reset skip state
  dispatch_skip_sec_ = 0.0;
  dispatch_skipped_prev_ = false;

  // --- All SDK calls succeeded — update per-arm tracking ---

  if (left_active)
  {
    for (size_t j = 0; j < kJointsPerArm; j++)
      left_last_written_[j] = hw_position_commands_[j];
    left_initial_write_pending_ = false;
  }

  if (right_active)
  {
    for (size_t j = 0; j < kJointsPerArm; j++)
      right_last_written_[j] = hw_position_commands_[kJointsPerArm + j];
    right_initial_write_pending_ = false;
  }

  return hardware_interface::return_type::OK;
}

// ============================================================================
// Mode switching — strict <joint>/<interface> parsing
// ============================================================================

hardware_interface::return_type MarvinBimanualArmHardware::prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces)
{
  auto count_arm = [&](const std::vector<std::string> & ifaces, int & left, int & right) {
    left = 0; right = 0;
    for (const auto & iface : ifaces)
    {
      auto slash = iface.find('/');
      if (slash == std::string::npos)
      {
        RCLCPP_ERROR(logger_, "Invalid interface '%s' — expected <joint>/<type>", iface.c_str());
        return false;
      }
      std::string joint = iface.substr(0, slash);
      std::string type  = iface.substr(slash + 1);

      if (type != "position")
      {
        RCLCPP_ERROR(logger_, "Unsupported interface type '%s' — only 'position' is allowed",
                     type.c_str());
        return false;
      }
      if (marvin_sdk_bridge::canonical_joint_names().find(joint) == marvin_sdk_bridge::canonical_joint_names().end())
      {
        RCLCPP_ERROR(logger_, "Unknown joint '%s'", joint.c_str());
        return false;
      }
      if (joint.find("_L") != std::string::npos) left++;
      else if (joint.find("_R") != std::string::npos) right++;
    }
    return true;
  };

  int start_left = 0, start_right = 0;
  int stop_left = 0, stop_right = 0;

  if (!count_arm(start_interfaces, start_left, start_right))
    return hardware_interface::return_type::ERROR;
  if (!count_arm(stop_interfaces,  stop_left,  stop_right))
    return hardware_interface::return_type::ERROR;

  // Validate: each arm must be claimed as a full set or not at all
  auto validate_arm = [&](int count, const char * arm_name) {
    if (count != 0 && count != static_cast<int>(kJointsPerArm))
    {
      RCLCPP_ERROR(logger_, "Partial %s arm claim (%d of %zu) — rejecting",
                   arm_name, count, kJointsPerArm);
      return false;
    }
    return true;
  };
  if (!validate_arm(start_left, "left"))   return hardware_interface::return_type::ERROR;
  if (!validate_arm(start_right, "right")) return hardware_interface::return_type::ERROR;
  if (!validate_arm(stop_left, "left"))    return hardware_interface::return_type::ERROR;
  if (!validate_arm(stop_right, "right"))  return hardware_interface::return_type::ERROR;

  const bool new_left_start = start_left > 0 && stop_left == 0;
  const bool new_right_start = start_right > 0 && stop_right == 0;
  const bool pure_left_stop = stop_left > 0 && start_left == 0;
  const bool pure_right_stop = stop_right > 0 && start_right == 0;
  if ((new_left_start && pure_right_stop) || (new_right_start && pure_left_stop))
  {
    RCLCPP_ERROR(logger_, "Cross-arm start/stop in one switch is unsupported — "
                 "stop the active arm before starting the other arm");
    return hardware_interface::return_type::ERROR;
  }

  pending_left_start_  = start_left;
  pending_right_start_ = start_right;
  pending_left_stop_   = stop_left;
  pending_right_stop_  = stop_right;

  RCLCPP_INFO(logger_, "prepare_command_mode_switch: start L=%d R=%d  stop L=%d R=%d",
              start_left, start_right, stop_left, stop_right);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MarvinBimanualArmHardware::perform_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/)
{
  std::lock_guard<std::mutex> lock(mode_switch_mutex_);
  bool left_started_this_switch = false;

  // ---- Capture baselines BEFORE any mode transition ----
  // Wait for a fresh frame (serial change) for each arm that is about to
  // start.  Baseline, commands, and last-written are all initialised from
  // this same frame so the first-write guard compares against the real
  // pre-transition pose.
  {
    DCSS pre_frame{};
    int saved_left_serial  = prev_left_serial_;
    int saved_right_serial = prev_right_serial_;

    bool left_pre_ok  = true;
    bool right_pre_ok = true;
    bool any_needed = (pending_left_start_ > 0 && pending_left_stop_ == 0)
                   || (pending_right_start_ > 0 && pending_right_stop_ == 0);

    if (any_needed)
    {
      // Retry for up to ~500 ms (10 × 50 ms) to get a frame with a new serial.
      for (int retry = 0; retry < 10; retry++)
      {
        if (!marvin_sdk_bridge::read_frame(pre_frame))
        {
          left_pre_ok = right_pre_ok = false;
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          continue;
        }
        left_pre_ok  = (pre_frame.m_Out[0].m_OutFrameSerial != 0
                        && pre_frame.m_Out[0].m_OutFrameSerial != saved_left_serial);
        right_pre_ok = (pre_frame.m_Out[1].m_OutFrameSerial != 0
                        && pre_frame.m_Out[1].m_OutFrameSerial != saved_right_serial);

        bool satisfied = true;
        if (pending_left_start_  > 0 && pending_left_stop_  == 0) satisfied &= left_pre_ok;
        if (pending_right_start_ > 0 && pending_right_stop_ == 0) satisfied &= right_pre_ok;
        if (satisfied) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }

    auto capture_arm = [&](int arm_idx, std::array<double, kJointsPerArm> & baseline,
                           size_t state_offset, bool & ok_flag) {
      if (!ok_flag) return;
      for (size_t j = 0; j < kJointsPerArm; j++)
      {
        double raw = pre_frame.m_Out[arm_idx].m_FB_Joint_Pos[j];
        if (std::isnan(raw) || std::isinf(raw)) { ok_flag = false; break; }
        double rad = marvin_sdk_bridge::deg_to_rad(raw);
        const auto & limits = marvin_sdk_bridge::kCanonicalJointLimits[j];
        if (rad < limits.lower_rad || rad > limits.upper_rad) { ok_flag = false; break; }
        baseline[j] = rad;
        hw_position_commands_[state_offset + j] = rad;
      }
    };

    if (pending_left_start_ > 0 && pending_left_stop_ == 0)
    {
      if (!left_pre_ok)
      {
        RCLCPP_ERROR(logger_, "Pre-capture: LEFT frame not usable — rejecting");
        pending_left_start_ = 0;
        return hardware_interface::return_type::ERROR;
      }
      capture_arm(0, left_activation_baseline_, 0, left_pre_ok);
      if (!left_pre_ok)
      {
        RCLCPP_ERROR(logger_, "Pre-capture: LEFT position is non-finite or outside limits — rejecting");
        pending_left_start_ = 0;
        return hardware_interface::return_type::ERROR;
      }
      for (size_t j = 0; j < kJointsPerArm; j++)
        left_last_written_[j] = left_activation_baseline_[j];
    }
    if (pending_right_start_ > 0 && pending_right_stop_ == 0)
    {
      if (!right_pre_ok)
      {
        RCLCPP_ERROR(logger_, "Pre-capture: RIGHT frame not usable — rejecting");
        pending_right_start_ = 0;
        return hardware_interface::return_type::ERROR;
      }
      capture_arm(1, right_activation_baseline_, kJointsPerArm, right_pre_ok);
      if (!right_pre_ok)
      {
        RCLCPP_ERROR(logger_, "Pre-capture: RIGHT position is non-finite or outside limits — rejecting");
        pending_right_start_ = 0;
        return hardware_interface::return_type::ERROR;
      }
      for (size_t j = 0; j < kJointsPerArm; j++)
        right_last_written_[j] = right_activation_baseline_[j];
    }

    if (any_needed)
      frame_data_ = pre_frame;
  }

  // A bimanual stop is one controller-manager switch and must use one SDK
  // transaction. Keep the software state unchanged unless both arms reach idle.
  if (pending_left_stop_ > 0 && pending_left_start_ == 0
      && pending_right_stop_ > 0 && pending_right_start_ == 0)
  {
    RCLCPP_INFO(logger_, "perform_command_mode_switch: BOTH arms exiting position mode...");
    if (!marvin_sdk_bridge::exit_position_modes(true, true))
    {
      RCLCPP_ERROR(logger_, "perform_command_mode_switch: bimanual exit-position-mode FAILED");
      pending_left_stop_ = 0;
      pending_right_stop_ = 0;
      return hardware_interface::return_type::ERROR;
    }
    left_position_active_ = false;
    right_position_active_ = false;
    left_initial_write_pending_ = false;
    right_initial_write_pending_ = false;
    pending_left_stop_ = 0;
    pending_right_stop_ = 0;
    RCLCPP_INFO(logger_, "perform_command_mode_switch: BOTH arms position INACTIVE");
    return hardware_interface::return_type::OK;
  }

  // ---- Left arm ----

  // Same-arm stop+start in one switch: keep active, skip state toggle.
  if (pending_left_start_ > 0 && pending_left_stop_ > 0)
  {
    RCLCPP_INFO(logger_, "perform_command_mode_switch: LEFT arm restart — keeping active");
    pending_left_start_ = 0;
    pending_left_stop_  = 0;
    left_stale_sec_ = 0.0;
    dispatch_skip_sec_     = 0.0;
    dispatch_skipped_prev_ = false;
    left_initial_write_pending_ = true;
    // Re-capture the complete handover state. Command interfaces retain the
    // previous controller's last value across a ros2_control switch, so seed
    // them from measured state before the new controller is activated.
    for (size_t j = 0; j < kJointsPerArm; j++)
    {
      left_activation_baseline_[j] = hw_position_states_[j];
      left_last_written_[j] = hw_position_states_[j];
      hw_position_commands_[j] = hw_position_states_[j];
    }
  }
  else if (pending_left_start_ > 0)
  {
    // Baseline, commands, and last-written were all set from the
    // pre-frame above — do NOT overwrite them from hw_position_states_.

    if (marvin_sdk_bridge::get_arm_error(frame_data_, 0) != 0)
    {
      RCLCPP_ERROR(logger_, "perform_command_mode_switch: LEFT arm error %d — rejecting start",
                   marvin_sdk_bridge::get_arm_error(frame_data_, 0));
      pending_left_start_ = 0;
      return hardware_interface::return_type::ERROR;
    }

    RCLCPP_INFO(logger_, "perform_command_mode_switch: LEFT arm entering position mode...");
    if (!marvin_sdk_bridge::enter_position_mode_left(marvin_sdk_bridge::kDefaultVelRatio,
                                                      marvin_sdk_bridge::kDefaultAccRatio))
    {
      RCLCPP_ERROR(logger_, "perform_command_mode_switch: LEFT arm position-mode transition FAILED");
      pending_left_start_ = 0;
      return hardware_interface::return_type::ERROR;
    }

    RCLCPP_INFO(logger_, "perform_command_mode_switch: LEFT arm position ACTIVE "
                "(baseline=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f])",
                left_activation_baseline_[0], left_activation_baseline_[1],
                left_activation_baseline_[2], left_activation_baseline_[3],
                left_activation_baseline_[4], left_activation_baseline_[5],
                left_activation_baseline_[6]);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    left_stale_sec_ = 0.0;  // fresh start — don't inherit pre-activation stall time
    dispatch_skip_sec_     = 0.0;
    dispatch_skipped_prev_ = false;
    left_position_active_ = true;
    left_initial_write_pending_ = true;
    left_started_this_switch = true;
    pending_left_start_ = 0;
  }
  else if (pending_left_stop_ > 0)
  {
    // Keep software state unchanged until the controller confirms idle.
    RCLCPP_INFO(logger_, "perform_command_mode_switch: LEFT arm exiting position mode...");
    if (!marvin_sdk_bridge::exit_position_mode_left())
    {
      RCLCPP_ERROR(logger_, "perform_command_mode_switch: LEFT arm exit-position-mode FAILED");
      pending_left_stop_ = 0;
      return hardware_interface::return_type::ERROR;
    }
    left_position_active_ = false;
    left_initial_write_pending_ = false;
    RCLCPP_INFO(logger_, "perform_command_mode_switch: LEFT arm position INACTIVE");
    pending_left_stop_ = 0;
  }

  // ---- Right arm ----

  if (pending_right_start_ > 0 && pending_right_stop_ > 0)
  {
    RCLCPP_INFO(logger_, "perform_command_mode_switch: RIGHT arm restart — keeping active");
    pending_right_start_ = 0;
    pending_right_stop_  = 0;
    right_stale_sec_ = 0.0;
    dispatch_skip_sec_     = 0.0;
    dispatch_skipped_prev_ = false;
    right_initial_write_pending_ = true;
    for (size_t j = 0; j < kJointsPerArm; j++)
    {
      right_activation_baseline_[j] = hw_position_states_[kJointsPerArm + j];
      right_last_written_[j] = hw_position_states_[kJointsPerArm + j];
      hw_position_commands_[kJointsPerArm + j] =
        hw_position_states_[kJointsPerArm + j];
    }
  }
  else if (pending_right_start_ > 0)
  {
    // Baseline, commands, and last-written were all set from the
    // pre-frame above — do NOT overwrite them.

    if (marvin_sdk_bridge::get_arm_error(frame_data_, 1) != 0)
    {
      // Roll back only an arm activated by this switch. An independently
      // running left controller must not be disturbed by a right-arm failure.
      if (left_started_this_switch)
      {
        RCLCPP_ERROR(logger_, "Rollback: RIGHT arm error — exiting LEFT position mode");
        if (marvin_sdk_bridge::exit_position_mode_left())
        {
          left_position_active_ = false;
          left_initial_write_pending_ = false;
        }
        else
        {
          RCLCPP_ERROR(logger_, "Rollback: LEFT arm failed to reach idle");
        }
      }
      RCLCPP_ERROR(logger_, "perform_command_mode_switch: RIGHT arm error %d — rejecting start",
                   marvin_sdk_bridge::get_arm_error(frame_data_, 1));
      pending_right_start_ = 0;
      return hardware_interface::return_type::ERROR;
    }

    RCLCPP_INFO(logger_, "perform_command_mode_switch: RIGHT arm entering position mode...");
    if (!marvin_sdk_bridge::enter_position_mode_right(marvin_sdk_bridge::kDefaultVelRatio,
                                                       marvin_sdk_bridge::kDefaultAccRatio))
    {
      if (left_started_this_switch)
      {
        RCLCPP_ERROR(logger_, "Rollback: RIGHT arm position-mode failed — "
                     "exiting LEFT position mode");
        if (marvin_sdk_bridge::exit_position_mode_left())
        {
          left_position_active_ = false;
          left_initial_write_pending_ = false;
        }
        else
        {
          RCLCPP_ERROR(logger_, "Rollback: LEFT arm failed to reach idle");
        }
      }
      RCLCPP_ERROR(logger_, "perform_command_mode_switch: RIGHT arm position-mode transition FAILED");
      pending_right_start_ = 0;
      return hardware_interface::return_type::ERROR;
    }

    RCLCPP_INFO(logger_, "perform_command_mode_switch: RIGHT arm position ACTIVE "
                "(baseline=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f])",
                right_activation_baseline_[0], right_activation_baseline_[1],
                right_activation_baseline_[2], right_activation_baseline_[3],
                right_activation_baseline_[4], right_activation_baseline_[5],
                right_activation_baseline_[6]);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    right_stale_sec_ = 0.0;  // fresh start — don't inherit pre-activation stall time
    dispatch_skip_sec_     = 0.0;
    dispatch_skipped_prev_ = false;
    right_position_active_ = true;
    right_initial_write_pending_ = true;
    pending_right_start_ = 0;
  }
  else if (pending_right_stop_ > 0)
  {
    RCLCPP_INFO(logger_, "perform_command_mode_switch: RIGHT arm exiting position mode...");
    if (!marvin_sdk_bridge::exit_position_mode_right())
    {
      RCLCPP_ERROR(logger_, "perform_command_mode_switch: RIGHT arm exit-position-mode FAILED");
      pending_right_stop_ = 0;
      return hardware_interface::return_type::ERROR;
    }
    right_position_active_ = false;
    right_initial_write_pending_ = false;
    RCLCPP_INFO(logger_, "perform_command_mode_switch: RIGHT arm position INACTIVE");
    pending_right_stop_ = 0;
  }

  return hardware_interface::return_type::OK;
}

// ============================================================================
// Read from hardware (internal)
// ============================================================================

bool MarvinBimanualArmHardware::read_frame_from_hardware(double period_sec)
{
  // Guard against a non-finite / negative period; 0 means "do not advance the
  // stale clock" (used by one-shot seeding reads).
  // Clamp to kMaxControlPeriodSec so a controller_manager overrun (which can
  // inject 500+ ms into the period) does not instantly trigger a stale-FATAL
  // on the first post-overrun read cycle.
  const double raw_dt = (std::isfinite(period_sec) && period_sec > 0.0) ? period_sec : 0.0;
  const double dt = std::min(raw_dt, marvin_sdk_bridge::kMaxControlPeriodSec);

  if (!marvin_sdk_bridge::read_frame(frame_data_))
  {
    RCLCPP_WARN_THROTTLE(logger_, *get_node()->get_clock(), 2000, "OnGetBuf false");
    left_stale_sec_  += dt;
    right_stale_sec_ += dt;
  }
  else
  {
    bool left_fresh = marvin_sdk_bridge::is_frame_fresh(frame_data_, 0, prev_left_serial_);
    bool right_fresh = marvin_sdk_bridge::is_frame_fresh(frame_data_, 1, prev_right_serial_);

    if (left_fresh)  left_stale_sec_  = 0.0;  else left_stale_sec_  += dt;
    if (right_fresh) right_stale_sec_ = 0.0;  else right_stale_sec_ += dt;
    dispatch_skip_sec_     = 0.0;
    dispatch_skipped_prev_ = false;
  }

  if (left_stale_sec_ >= stale_warn_sec_ || right_stale_sec_ >= stale_warn_sec_)
  {
    RCLCPP_WARN_THROTTLE(logger_, *get_node()->get_clock(), 1000,
                "Stale frames: L=%.0fms R=%.0fms (warn=%.0fms, error=%.0fms)",
                left_stale_sec_ * 1e3, right_stale_sec_ * 1e3,
                stale_warn_sec_ * 1e3, stale_error_sec_ * 1e3);
  }
  if (left_stale_sec_ >= stale_error_sec_ || right_stale_sec_ >= stale_error_sec_)
  {
    RCLCPP_ERROR(logger_, "FATAL: stale frames L=%.0fms R=%.0fms — returning ERROR",
                 left_stale_sec_ * 1e3, right_stale_sec_ * 1e3);
    return false;
  }

  // Log arm errors
  {
    int le = marvin_sdk_bridge::get_arm_error(frame_data_, 0);
    int re = marvin_sdk_bridge::get_arm_error(frame_data_, 1);
    if (le) RCLCPP_WARN_THROTTLE(logger_, *get_node()->get_clock(), 5000, "Left  err: %d", le);
    if (re) RCLCPP_WARN_THROTTLE(logger_, *get_node()->get_clock(), 5000, "Right err: %d", re);
  }

  copy_arm_state_to_buffers(0, 0);
  copy_arm_state_to_buffers(1, kJointsPerArm);
  return true;
}

void MarvinBimanualArmHardware::copy_arm_state_to_buffers(int arm_idx, size_t offset)
{
  // Per-joint conversion from raw SDK frame to ROS state buffers.
  // Freshness is enforced via serial tracking in read_frame_from_hardware();
  // this function only guards against NaN / Inf in individual fields.

  for (size_t i = 0; i < kJointsPerArm; i++)
  {
    size_t d = offset + i;

    // Vendor feedback torque. The SDK does not document its physical unit or
    // scaling, so this remains provisional and must not drive force control.
    {
      double raw = frame_data_.m_Out[arm_idx].m_FB_Joint_SToq[i];
      if (!std::isnan(raw) && !std::isinf(raw))
        hw_effort_states_[d] = raw;
      else
        RCLCPP_WARN_THROTTLE(logger_, *get_node()->get_clock(), 2000,
                             "Arm%d J%zu effort NaN/Inf", arm_idx, i + 1);
    }

    // Velocity: deg/s → rad/s
    {
      double raw_vel = frame_data_.m_Out[arm_idx].m_FB_Joint_Vel[i];
      if (!std::isnan(raw_vel) && !std::isinf(raw_vel))
        hw_velocity_states_[d] = marvin_sdk_bridge::deg_to_rad(raw_vel);
      else
        RCLCPP_WARN_THROTTLE(logger_, *get_node()->get_clock(), 2000,
                             "Arm%d J%zu vel NaN/Inf", arm_idx, i + 1);
    }

    // Position: deg → rad, guarded by NaN/Inf only (serial freshness is
    // checked independently by read_frame_from_hardware).
    double raw_pos = frame_data_.m_Out[arm_idx].m_FB_Joint_Pos[i];
    if (!std::isnan(raw_pos) && !std::isinf(raw_pos))
      hw_position_states_[d] = marvin_sdk_bridge::deg_to_rad(raw_pos);
    else
      RCLCPP_WARN_THROTTLE(logger_, *get_node()->get_clock(), 2000,
                           "Arm%d J%zu pos NaN/Inf", arm_idx, i + 1);
  }
}

}  // namespace marvin_hardware_interface

PLUGINLIB_EXPORT_CLASS(
    marvin_hardware_interface::MarvinBimanualArmHardware,
    hardware_interface::SystemInterface)
