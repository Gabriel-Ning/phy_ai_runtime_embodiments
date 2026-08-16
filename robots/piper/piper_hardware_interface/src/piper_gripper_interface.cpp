// Copyright 2026 Piper ros2_control integration
// SPDX-License-Identifier: Apache-2.0

#include "piper_hardware_interface/piper_gripper_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <string>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace piper_hardware_interface
{

namespace
{
constexpr auto kFeedbackTimeout = std::chrono::milliseconds{100};
constexpr double kDefaultCommandForceN = 1.0;
constexpr double kFingerTravelPerOpeningWidth = 0.5;
constexpr double kMaxOpeningWidthM = 0.08;
}  // namespace

// ── logging ───────────────────────────────────────────────────────────────────

rclcpp::Logger PiperGripperInterface::getLogger()
{
  static rclcpp::Logger lg = rclcpp::get_logger("PiperGripperInterface");
  return lg;
}

// ── on_init ───────────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn PiperGripperInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & info = params.hardware_info;

  if (info.joints.size() != 1u)
  {
    RCLCPP_ERROR(
      getLogger(), "PiperGripperInterface expects exactly 1 joint, got %zu",
      info.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_name_ = info.joints[0].name;

  const auto can_it = info.hardware_parameters.find("can_interface");
  if (can_it == info.hardware_parameters.end() || can_it->second.empty())
  {
    RCLCPP_ERROR(getLogger(), "Missing required hardware parameter 'can_interface'");
    return hardware_interface::CallbackReturn::ERROR;
  }
  can_interface_ = can_it->second;

  const auto home_it = info.hardware_parameters.find("home_on_activate");
  if (home_it != info.hardware_parameters.end())
  {
    home_on_activate_ = (home_it->second == "true" || home_it->second == "True" ||
                         home_it->second == "1");
  }

  RCLCPP_INFO(
    getLogger(),
    "PiperGripperInterface init: joint='%s' can='%s' home=%d",
    joint_name_.c_str(), can_interface_.c_str(), home_on_activate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_configure ──────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn PiperGripperInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  try
  {
    // SocketCAN link setup is a host deployment responsibility. The SDK default
    // leaves interface initialisation disabled.
    gripper_ = std::make_unique<piper::Gripper>(can_interface_);
  }
  catch (const std::exception & ex)
  {
    RCLCPP_ERROR(getLogger(), "Failed to construct piper::Gripper: %s", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_activate ───────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn PiperGripperInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!gripper_)
  {
    RCLCPP_ERROR(getLogger(), "Gripper not configured; cannot activate");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Optionally home the gripper (close to mechanical stop and zero position).
  if (home_on_activate_)
  {
    RCLCPP_INFO(getLogger(), "Homing gripper (closing to mechanical zero)...");
    try
    {
      const bool ok = gripper_->homing();
      if (!ok)
      {
        RCLCPP_WARN(getLogger(), "Gripper homing() returned false (may still be homed)");
      }
      else
      {
        RCLCPP_INFO(getLogger(), "Gripper homing complete");
      }
    }
    catch (const std::exception & ex)
    {
      RCLCPP_WARN(getLogger(), "Gripper homing() threw: %s", ex.what());
    }
  }

  // Seed state/command from the current hardware position.
  try
  {
    const auto state = gripper_->readOnce();
    if (!state.valid || !std::isfinite(state.width))
    {
      RCLCPP_ERROR(getLogger(), "Cannot activate from invalid Piper gripper feedback");
      return hardware_interface::CallbackReturn::ERROR;
    }
    const double finger_position = state.width * kFingerTravelPerOpeningWidth;
    hw_position_ = finger_position;
    hw_position_command_ = finger_position;
    last_sent_position_ = finger_position;
  }
  catch (const std::exception & ex)
  {
    RCLCPP_ERROR(getLogger(), "Initial gripper readOnce() failed: %s", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_velocity_ = 0.0;

  RCLCPP_INFO(
    getLogger(), "PiperGripperInterface activated (initial width = %.4f m)", hw_position_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_deactivate ─────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn PiperGripperInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (gripper_)
  {
    try { gripper_->stop(); }
    catch (const std::exception & ex)
    {
      RCLCPP_WARN(getLogger(), "gripper_->stop() threw: %s", ex.what());
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_cleanup ────────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn PiperGripperInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  gripper_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_error ──────────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn PiperGripperInterface::on_error(
  const rclcpp_lifecycle::State &)
{
  gripper_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── export_state_interfaces ───────────────────────────────────────────────────

std::vector<hardware_interface::StateInterface>
PiperGripperInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  si.emplace_back(joint_name_, hardware_interface::HW_IF_POSITION, &hw_position_);
  si.emplace_back(joint_name_, hardware_interface::HW_IF_VELOCITY, &hw_velocity_);
  return si;
}

// ── export_command_interfaces ─────────────────────────────────────────────────

std::vector<hardware_interface::CommandInterface>
PiperGripperInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  ci.emplace_back(joint_name_, hardware_interface::HW_IF_POSITION, &hw_position_command_);
  return ci;
}

// ── read ──────────────────────────────────────────────────────────────────────

hardware_interface::return_type PiperGripperInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!gripper_)
  {
    return hardware_interface::return_type::ERROR;
  }

  try
  {
    const auto state = gripper_->readLatest();
    const auto now = std::chrono::steady_clock::now();
    const bool timed_out = !state.valid ||
      state.feedback_receive_time == std::chrono::steady_clock::time_point{} ||
      now - state.feedback_receive_time > kFeedbackTimeout;
    if (timed_out || !std::isfinite(state.width))
    {
      RCLCPP_ERROR(
        getLogger(),
        "Piper gripper feedback stale/invalid (valid=%d sequence=%llu timeout=%lld ms)",
        state.valid, static_cast<unsigned long long>(state.feedback_sequence),
        static_cast<long long>(kFeedbackTimeout.count()));
      return hardware_interface::return_type::ERROR;
    }
    hw_position_ = state.width * kFingerTravelPerOpeningWidth;
  }
  catch (const std::exception & ex)
  {
    RCLCPP_ERROR(getLogger(), "Piper gripper readLatest() failed: %s", ex.what());
    return hardware_interface::return_type::ERROR;
  }

  hw_velocity_ = 0.0;
  return hardware_interface::return_type::OK;
}

// ── write ─────────────────────────────────────────────────────────────────────

hardware_interface::return_type PiperGripperInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!std::isfinite(hw_position_command_))
  {
    RCLCPP_ERROR(getLogger(), "write(): non-finite gripper position command — skipping");
    return hardware_interface::return_type::ERROR;
  }

  if (hw_position_command_ == last_sent_position_)
  {
    return hardware_interface::return_type::OK;
  }

  try
  {
    double opening_width = hw_position_command_ / kFingerTravelPerOpeningWidth;
    if (!std::isfinite(opening_width))
    {
      RCLCPP_ERROR(getLogger(), "write(): non-finite opening width — skipping");
      return hardware_interface::return_type::ERROR;
    }
    const double clamped = std::clamp(opening_width, 0.0, kMaxOpeningWidthM);
    if (clamped != opening_width)
    {
      RCLCPP_WARN_THROTTLE(
        getLogger(), *get_clock(), 1000,
        "Clamping gripper opening width %.5f -> %.5f m", opening_width, clamped);
      opening_width = clamped;
    }
    if (!gripper_ || !gripper_->commandWidth(opening_width, kDefaultCommandForceN))
    {
      RCLCPP_ERROR(getLogger(), "Failed to send Piper gripper position command");
      return hardware_interface::return_type::ERROR;
    }
    last_sent_position_ = hw_position_command_;
  }
  catch (const std::exception & ex)
  {
    RCLCPP_ERROR(getLogger(), "Piper gripper commandWidth() failed: %s", ex.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace piper_hardware_interface

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
PLUGINLIB_EXPORT_CLASS(
    piper_hardware_interface::PiperGripperInterface,
    hardware_interface::SystemInterface)
