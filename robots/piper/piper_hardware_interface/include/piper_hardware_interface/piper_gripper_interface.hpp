// Copyright 2026 Piper ros2_control integration
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <piper/gripper.h>
#include <piper/gripper_state.h>

namespace piper_hardware_interface
{

/// ros2_control SystemInterface for the Piper gripper.
///
/// Uses libpiper's streaming API: readLatest() returns the cached 0x2A8 sample
/// immediately and commandWidth() sends one target frame without waiting for
/// the gripper to reach it.
///
/// Exposes one joint (gripper_joint1) with:
///   - command interface: position   [m]  (one-finger prismatic travel)
///   - state  interface: position    [m]  (SDK full opening width / 2)
///   - state  interface: velocity    [m/s] (always 0 — hardware not reported)
///
/// Intended for forward-position streaming. Any controller that claims the
/// single position interface may use the same hardware contract.
class PiperGripperInterface final : public hardware_interface::SystemInterface
{
public:
  PiperGripperInterface() = default;
  ~PiperGripperInterface() override = default;

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

  std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static rclcpp::Logger getLogger();

  // ── Hardware parameters ────────────────────────────────────────────────────
  std::string can_interface_;
  bool        home_on_activate_{true};  // call homing() on activate to zero the gripper

  // ── libpiper gripper ───────────────────────────────────────────────────────
  std::unique_ptr<piper::Gripper> gripper_;

  // ── Joint buffers (exported to ros2_control) ───────────────────────────────
  std::string joint_name_;
  double      hw_position_{0.0};          // state: position  [m]
  double      hw_velocity_{0.0};          // state: velocity  [m/s]  (always 0)
  double      hw_position_command_{0.0};  // command: position [m]

  // Last target successfully sent. Exact equality suppresses controller-manager
  // repeats without dropping small policy updates through a deadband.
  double last_sent_position_{-1.0};
};

}  // namespace piper_hardware_interface
