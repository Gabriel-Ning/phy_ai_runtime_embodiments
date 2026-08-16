// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
//
// Marvin CCS M6 bimanual arm hardware interface.
// Connects to the robot controller via TJ/FX SDK over TCP/IP,
// reads dual-arm joint state (14 DOF), and dispatches position commands
// per-arm via JTC.
//
// SDK details are isolated in marvin_sdk_bridge.hpp / marvin_sdk_bridge.cpp.
// This file owns the ros2_control lifecycle, mode switching, and RT read/write.

#ifndef MARVIN_HARDWARE_INTERFACE__MARVIN_HARDWARE_INTERFACE_HPP_
#define MARVIN_HARDWARE_INTERFACE__MARVIN_HARDWARE_INTERFACE_HPP_

#include <array>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "marvin_hardware_interface/marvin_sdk_bridge.hpp"

namespace marvin_hardware_interface
{

class MarvinBimanualArmHardware final : public hardware_interface::SystemInterface
{
public:
  MarvinBimanualArmHardware() = default;
  ~MarvinBimanualArmHardware() override = default;

  // Lifecycle
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  // State and command interface export
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Mode switching
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  // Real-time read / write loop
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // --- Read helpers ---
  // period_sec drives the time-based stale accumulators; pass 0.0 to read a
  // frame without advancing the stale clock (e.g. one-shot seeding reads).
  bool read_frame_from_hardware(double period_sec);
  void copy_arm_state_to_buffers(int arm_idx, size_t offset);

  // --- Write helpers (per-arm) ---
  bool validate_arm_commands(size_t offset, bool initial_write_pending,
                             const std::array<double, marvin_sdk_bridge::kJointsPerArm> & last_written,
                             const std::array<double, marvin_sdk_bridge::kJointsPerArm> & baseline);
  bool dispatch_arm_command(int arm_idx, size_t offset);

  // --- Stable parameters (from URDF hardware <param> tags) ---
  std::string robot_ip_{"10.19.0.191"};
  bool use_fake_hardware_{true};
  // Time-based stale thresholds (seconds) — independent of update_rate.
  double stale_warn_sec_{marvin_sdk_bridge::kDefaultStaleWarnSec};
  double stale_error_sec_{marvin_sdk_bridge::kDefaultStaleErrorSec};
  // Velocity-based per-cycle motion guard (rad/s).
  double max_joint_velocity_{marvin_sdk_bridge::kDefaultMaxJointVelocityRadPerSec};

  // --- Joint state buffers ---
  std::vector<double> hw_position_states_;
  std::vector<double> hw_velocity_states_;
  std::vector<double> hw_effort_states_;

  // --- Command buffers ---
  std::vector<double> hw_position_commands_;
  std::vector<double> hw_commands_deg_buffer_;

  // --- Per-arm write safety ---
  // Activation baseline: captured from measured state at arm activation time.
  std::array<double, marvin_sdk_bridge::kJointsPerArm> left_activation_baseline_{};
  std::array<double, marvin_sdk_bridge::kJointsPerArm> right_activation_baseline_{};

  // Last successfully dispatched command (for per-cycle delta guard).
  std::array<double, marvin_sdk_bridge::kJointsPerArm> left_last_written_{};
  std::array<double, marvin_sdk_bridge::kJointsPerArm> right_last_written_{};

  // First-write guard: true until the first successful dispatch after activation.
  bool left_initial_write_pending_{false};
  bool right_initial_write_pending_{false};

  // Joint names (from URDF, in order)
  std::vector<std::string> joint_names_;

  // SDK data frame
  DCSS frame_data_{};

  // Stale-frame tracking (per-arm). Freshness is detected via serial change;
  // the accumulators track elapsed time without a fresh frame (seconds).
  int prev_left_serial_{-1};
  int prev_right_serial_{-1};
  double left_stale_sec_{0.0};
  double right_stale_sec_{0.0};

  // Per-arm active flags (set by mode switch)
  std::atomic<bool> left_position_active_{false};
  std::atomic<bool> right_position_active_{false};

  // Dispatch-skip tracking (T2/T3)
  double dispatch_skip_sec_{0.0};
  bool dispatch_skipped_prev_{false};

  // Mode-switch pending counters
  int pending_left_start_{0};
  int pending_right_start_{0};
  int pending_left_stop_{0};
  int pending_right_stop_{0};
  std::mutex mode_switch_mutex_;

  // Connection state
  bool hardware_connected_{false};

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("MarvinBimanualArmHardware")};
};

}  // namespace marvin_hardware_interface

#endif  // MARVIN_HARDWARE_INTERFACE__MARVIN_HARDWARE_INTERFACE_HPP_
