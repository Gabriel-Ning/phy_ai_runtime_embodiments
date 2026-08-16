// Copyright 2026 Piper ros2_control integration
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <piper/robot.h>
#include <piper/robot_state.h>
#include <realtime_tools/realtime_buffer.hpp>

namespace piper_hardware_interface
{

class PiperHardwareInterface final : public hardware_interface::SystemInterface
{
public:
  PiperHardwareInterface() = default;
  ~PiperHardwareInterface() override = default;

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

  /// Called by CM when read() or write() returns ERROR. Must release hardware
  /// without throwing — robot_->stop() may already be invalid at this point.
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface>  export_state_interfaces()   override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ── Logging ────────────────────────────────────────────────────────────────
  static rclcpp::Logger getLogger()
  {
    static rclcpp::Logger lg = rclcpp::get_logger("PiperHardwareInterface");
    return lg;
  }

  /// Stop the active control and robot without throwing.
  /// Resetting active_control_ stops the robot through its destructor; robot_->stop()
  /// is called explicitly only when no active control exists.
  /// Caller must hold mode_switch_mutex_.
  void stopRobotSafe() noexcept;

  // ── Hardware parameters ────────────────────────────────────────────────────
  std::string can_interface_;
  // Joint / opaque-state name prefix (e.g. "left_", "right_"). Empty for
  // single-arm unprefixed models. Mirrors franka_hardware dual-arm naming.
  std::string prefix_;
  // ros2_control hardware parameter passed through the robot description.
  // Keep zero as the default so the effort interface remains torque-transparent.
  double mit_kd_effort_damping_{0.0};

  // ── libpiper objects ───────────────────────────────────────────────────────
  std::unique_ptr<piper::Robot> robot_;

  // Protects robot_ and active_control_ across lifecycle, mode-switch, read,
  // and write callbacks. read()/write() use try_lock and skip rather than block.
  //
  //   - robot_->readLatest() reads a non-blocking coherent CAN snapshot.
  //
  //   - active_control_->writeOnce()  posts to a target buffer protected by
  //     Robot::Impl::target_mutex_; the 500 Hz internal thread consumes it.
  //     The call returns immediately.
  //
  std::mutex mode_switch_mutex_;

  std::unique_ptr<piper::ActiveControlBase> active_control_;

  // ── Joint metadata ─────────────────────────────────────────────────────────
  std::vector<std::string> joint_names_;

  // ── Joint state buffers ────────────────────────────────────────────────────
  std::vector<double> hw_positions_{0, 0, 0, 0, 0, 0};
  std::vector<double> hw_velocities_{0, 0, 0, 0, 0, 0};
  std::vector<double> hw_efforts_{0, 0, 0, 0, 0, 0};

  // ── Joint command buffers (one per supported mode) ─────────────────────────
  std::vector<double> hw_position_commands_{0, 0, 0, 0, 0, 0};
  std::vector<double> hw_velocity_commands_{0, 0, 0, 0, 0, 0};
  std::vector<double> hw_effort_commands_{0, 0, 0, 0, 0, 0};

  // ── Extended state (opaque pointer slots, mirrors franka_hardware convention) ─
  //
  // cartesian_pose_state_[i] is exported as StateInterface("{prefix}piper_i", "cartesian_pose_state").
  // hw_piper_robot_state_addr_ stores a pointer to hw_piper_robot_state_ so that
  // downstream semantic components can retrieve the full RobotState via:
  //
  //   auto* ptr = *reinterpret_cast<piper::RobotState**>(handle.get_value_ptr());
  //
  // The StateInterface slot holds reinterpret_cast<double*>(&rt_robot_state_buffer_ptr_),
  // i.e. the address of the pointer cast to double*. This is
  // identical to the franka_hardware::FrankaHardwareInterface convention.
  std::array<double, 16> cartesian_pose_state_{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  double robot_time_state_{0.0};

  realtime_tools::RealtimeBuffer<piper::RobotState> rt_robot_state_buffer_;
  realtime_tools::RealtimeBuffer<piper::RobotState>* rt_robot_state_buffer_ptr_ = &rt_robot_state_buffer_;

  // ── Command mode flags ─────────────────────────────────────────────────────
  // claimed  = CM has requested this interface via prepare_command_mode_switch
  //            (only accessed from CM service thread — no race, plain bool).
  // running  = the corresponding libpiper ActiveControl is started.
  //            Written under mode_switch_mutex_ (perform / deactivate / error),
  //            READ from read() / write() on the real-time thread → atomic.
  bool position_interface_claimed_{false};
  std::atomic<bool> position_interface_running_{false};
  bool velocity_interface_claimed_{false};
  std::atomic<bool> velocity_interface_running_{false};
  bool effort_interface_claimed_{false};
  std::atomic<bool> effort_interface_running_{false};

  // ── First-cycle write guards (mirrors Franka's first_position_update_) ─────
  // Written in perform_command_mode_switch (service thread) and cleared in
  // read() (real-time thread) → must be atomic.
  std::atomic<bool> first_position_write_{true};
  std::atomic<bool> first_velocity_write_{true};
  std::atomic<bool> first_effort_write_{true};

  // ── Diagnostics: skip counters for mode-switch-induced frame drops ───────────
  // Incremented when try_lock fails in read()/write(); reset on success.
  // Used for throttled logging — avoids flooding the log at control rate.
  std::atomic<uint32_t> read_skip_count_{0};
  std::atomic<uint32_t> write_skip_count_{0};
  bool incoherent_feedback_logged_{false};

  // -- URDF-derived command interface registry -------------------------------
  // Populated in on_init from info.joints[*].command_interfaces.
  // Used for O(1) lookup in prepare_command_mode_switch.
  std::unordered_set<std::string> exported_command_interfaces_;

  // ── Command storage map (used by export_command_interfaces) ───────────────
  // Maps interface type name → reference to the corresponding command buffer.
  // export_command_interfaces() iterates info_.joints and uses this map so
  // that only URDF-declared interfaces are registered with the CM — consistent
  // with the franka_hardware approach.
  std::map<std::string, std::vector<double> &> command_interface_map_{
      {hardware_interface::HW_IF_POSITION, hw_position_commands_},
      {hardware_interface::HW_IF_VELOCITY, hw_velocity_commands_},
      {hardware_interface::HW_IF_EFFORT,   hw_effort_commands_}};
};

}  // namespace piper_hardware_interface
