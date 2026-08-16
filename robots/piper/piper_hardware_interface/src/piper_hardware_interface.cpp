// Copyright 2026 Piper ros2_control integration
// SPDX-License-Identifier: Apache-2.0

#include "piper_hardware_interface/piper_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <piper/mit_config.h>
#include <piper/realtime.h>
#include <pluginlib/class_list_macros.hpp>

namespace
{

constexpr char kCanInterfaceParam[] = "can_interface";
constexpr char kPrefixParam[] = "prefix";
constexpr char kMitKdEffortDampingParam[] = "mit_kd_effort_damping";
constexpr double kDefaultMitKdEffortDamping = 0.0;
constexpr auto kFeedbackTimeout = std::chrono::milliseconds{100};

/// Returns true if any element of the range is non-finite (NaN or Inf).
template <typename Container>
bool hasInfinite(const Container & c)
{
  return std::any_of(c.begin(), c.end(), [](double v) { return !std::isfinite(v); });
}

std::optional<double> parseFiniteNonNegativeDouble(const std::string & value)
{
  try
  {
    size_t parsed_chars = 0;
    const double parsed = std::stod(value, &parsed_chars);
    if (parsed_chars != value.size() || !std::isfinite(parsed) || parsed < 0.0)
    {
      return std::nullopt;
    }
    return parsed;
  }
  catch (const std::exception &)
  {
    return std::nullopt;
  }
}

}  // namespace

namespace piper_hardware_interface
{

// ── on_init ──────────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn PiperHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & info = params.hardware_info;

  // [Issue 6] Validate joint count.
  if (info.joints.size() != static_cast<std::size_t>(piper::kNumJoints))
  {
    RCLCPP_ERROR(
      getLogger(), "Expected exactly %zu joints, got %zu",
      static_cast<std::size_t>(piper::kNumJoints), info.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Collect names and validate per-joint command_interface declarations.
  // [Issue 6] Unknown interface types would cause map::at() to throw in
  // export_command_interfaces(); reject them here with a clear error.
  static const std::unordered_set<std::string> kKnownCmdIf{
      hardware_interface::HW_IF_POSITION,
      hardware_interface::HW_IF_VELOCITY,
      hardware_interface::HW_IF_EFFORT};

  joint_names_.clear();
  joint_names_.reserve(info.joints.size());
  exported_command_interfaces_.clear();

  for (const auto & joint : info.joints)
  {
    joint_names_.push_back(joint.name);
    for (const auto & cmd : joint.command_interfaces)
    {
      if (kKnownCmdIf.count(cmd.name) == 0)
      {
        RCLCPP_ERROR(
          getLogger(), "Unknown command_interface '%s' on joint '%s'",
          cmd.name.c_str(), joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      exported_command_interfaces_.insert(joint.name + "/" + cmd.name);
    }
  }

  // Hardware parameters.
  const auto can_it = info.hardware_parameters.find(kCanInterfaceParam);
  if (can_it == info.hardware_parameters.end() || can_it->second.empty())
  {
    RCLCPP_ERROR(getLogger(), "Missing required hardware parameter '%s'", kCanInterfaceParam);
    return hardware_interface::CallbackReturn::ERROR;
  }
  can_interface_ = can_it->second;

  // Dual-arm opaque state interfaces must be uniquely named in one CM.
  // Prefer the URDF hardware param; else derive from the first joint name
  // (e.g. "left_joint1" -> "left_").
  const auto prefix_it = info.hardware_parameters.find(kPrefixParam);
  if (prefix_it != info.hardware_parameters.end())
  {
    prefix_ = prefix_it->second;
  }
  else if (!joint_names_.empty())
  {
    const auto pos = joint_names_.front().find("joint");
    prefix_ = (pos != std::string::npos) ? joint_names_.front().substr(0, pos) : "";
  }

  mit_kd_effort_damping_ = kDefaultMitKdEffortDamping;
  const auto mit_kd_it = info.hardware_parameters.find(kMitKdEffortDampingParam);
  if (mit_kd_it != info.hardware_parameters.end() && !mit_kd_it->second.empty())
  {
    const auto parsed = parseFiniteNonNegativeDouble(mit_kd_it->second);
    if (!parsed.has_value())
    {
      RCLCPP_ERROR(
        getLogger(),
        "Invalid hardware parameter '%s'='%s'; expected a finite value >= 0",
        kMitKdEffortDampingParam, mit_kd_it->second.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    mit_kd_effort_damping_ = parsed.value();
  }

  // Initialise buffers.
  hw_positions_.assign(piper::kNumJoints, 0.0);
  hw_velocities_.assign(piper::kNumJoints, 0.0);
  hw_efforts_.assign(piper::kNumJoints, 0.0);
  hw_position_commands_.assign(piper::kNumJoints, 0.0);
  hw_velocity_commands_.assign(piper::kNumJoints, 0.0);
  hw_effort_commands_.assign(piper::kNumJoints, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_configure ─────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn PiperHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  try
  {
    const bool realtime_kernel = piper::hasRealtimeKernel();
    robot_ = std::make_unique<piper::Robot>(
      can_interface_, realtime_kernel ? piper::RealtimeConfig::kEnforce
                                      : piper::RealtimeConfig::kIgnore);
    RCLCPP_INFO(
      getLogger(), "libpiper control scheduling: %s",
      realtime_kernel ? "realtime enforced (SCHED_FIFO priority 80)"
                      : "normal scheduling (non-RT kernel)");

    piper::MitGains gains{};
    gains.kd_effort_damping = mit_kd_effort_damping_;
    robot_->setMitGains(gains);
    RCLCPP_INFO(
      getLogger(), "Configured MIT effort damping kd=%.6f", mit_kd_effort_damping_);
  }
  catch (const std::exception & ex)
  {
    RCLCPP_ERROR(getLogger(), "Failed to configure piper::Robot: %s", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_activate ──────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn PiperHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!robot_)
  {
    RCLCPP_ERROR(getLogger(), "Robot not configured; cannot activate");
    return hardware_interface::CallbackReturn::ERROR;
  }

  try
  {
    robot_->enableArm();

    // First readOnce() may block ≤500 ms waiting for CAN joint feedback to
    // arrive (initial_joint_feedback_waited_ one-shot inside Robot::Impl).
    // Subsequent calls read from the CAN receive cache and are non-blocking.
    const auto snapshot = robot_->readOnce();
    if (!snapshot.valid || !snapshot.coherent)
    {
      RCLCPP_ERROR(
        getLogger(),
        "Cannot activate from invalid Piper feedback (valid=%d coherent=%d sequence=%llu)",
        snapshot.valid, snapshot.coherent,
        static_cast<unsigned long long>(snapshot.feedback_sequence));
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (size_t i = 0; i < static_cast<size_t>(piper::kNumJoints); ++i)
    {
      hw_positions_[i] = snapshot.q[i];
      hw_velocities_[i] = snapshot.dq[i];
      hw_efforts_[i] = snapshot.tau_J[i];

      // Seed command buffers to the current pose so that the first write
      // (after first_XXX_write_ is cleared in read()) starts from a safe
      // position rather than zeros.
      hw_position_commands_[i] = snapshot.q[i];
      hw_velocity_commands_[i] = 0.0;
      hw_effort_commands_[i]   = 0.0;
    }

    cartesian_pose_state_ = snapshot.O_T_EE;
    robot_time_state_     = snapshot.time.toSec();
    rt_robot_state_buffer_.writeFromNonRT(snapshot);
  }
  catch (const std::exception & ex)
  {
    RCLCPP_ERROR(getLogger(), "on_activate failed: %s", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_deactivate ────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn PiperHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  {
    std::lock_guard<std::mutex> lock(mode_switch_mutex_);
    stopRobotSafe();

    // Reset mode flags so that perform_command_mode_switch re-initialises
    // cleanly if the interface is re-activated.
    position_interface_running_.store(false);
    velocity_interface_running_.store(false);
    effort_interface_running_.store(false);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_cleanup ───────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn PiperHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(mode_switch_mutex_);
  stopRobotSafe();
  robot_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_error ─────────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn PiperHardwareInterface::on_error(
  const rclcpp_lifecycle::State &)
{
  // [Issue 7] Fatal clean-up: do NOT call robot_->stop() here — the hardware
  // may already be in an unrecoverable state. Simply drop the objects.
  std::lock_guard<std::mutex> lock(mode_switch_mutex_);
  active_control_.reset();
  robot_.reset();

  position_interface_running_.store(false);
  velocity_interface_running_.store(false);
  effort_interface_running_.store(false);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── export_state_interfaces ───────────────────────────────────────────────────

std::vector<hardware_interface::StateInterface> PiperHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Per-joint standard interfaces.
  for (size_t i = 0; i < static_cast<size_t>(piper::kNumJoints); ++i)
  {
    state_interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    state_interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]);
  }

  // [Issue 4] Opaque robot_state pointer — convention identical to
  // franka_hardware::FrankaHardwareInterface:
  //   The StateInterface slot stores reinterpret_cast<double*>(&addr_),
  //   where addr_ is a piper::RobotState*. Consumers retrieve the full
  //   state via:
  //     auto* p = *reinterpret_cast<piper::RobotState**>(handle.get_value_ptr());
  // Names are prefixed (left_piper / right_piper) so two arms can share one CM.
  const std::string opaque = prefix_ + "piper";
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      opaque, "robot_state",
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
      reinterpret_cast<double *>(&rt_robot_state_buffer_ptr_)));

  // Cartesian pose — 16 scalars exported as ("{prefix}piper_i", "cartesian_pose_state").
  for (size_t i = 0; i < 16; ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        opaque + "_" + std::to_string(i), "cartesian_pose_state",
        &cartesian_pose_state_.at(i)));
  }

  // Scalar robot time.
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(opaque, "robot_time", &robot_time_state_));

  return state_interfaces;
}

// ── export_command_interfaces ─────────────────────────────────────────────────

std::vector<hardware_interface::CommandInterface>
PiperHardwareInterface::export_command_interfaces()
{
  // [Issue 2] Iterate the URDF-declared interfaces instead of hard-coding all
  // three modes. This matches franka_hardware: only interfaces listed in the
  // URDF ros2_control block are registered with the Controller Manager.
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    for (const auto & cmd : info_.joints[i].command_interfaces)
    {
      command_interfaces.emplace_back(
        info_.joints[i].name, cmd.name,
        &command_interface_map_.at(cmd.name)[i]);
    }
  }

  return command_interfaces;
}

// ── prepare_command_mode_switch ───────────────────────────────────────────────

hardware_interface::return_type PiperHardwareInterface::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  bool next_position_claimed = position_interface_claimed_;
  bool next_velocity_claimed = velocity_interface_claimed_;
  bool next_effort_claimed = effort_interface_claimed_;

  // Count start/stop per interface type using only URDF-exported interfaces.
  auto count_type = [this](const std::vector<std::string> & interfaces,
                           const std::string & type) -> size_t
  {
    size_t n = 0;
    for (const auto & iface : interfaces)
    {
      if (exported_command_interfaces_.count(iface) > 0)
      {
        const size_t slash = iface.find('/');
        if (slash != std::string::npos && iface.substr(slash + 1) == type)
        {
          ++n;
        }
      }
    }
    return n;
  };

  // Helper: validate a partial start/stop count (must be 0 or all-joints).
  auto validate = [this](size_t n, bool & claimed, bool start) -> bool
  {
    if (n == piper::kNumJoints)
    {
      claimed = start;
      return true;
    }
    return n == 0;  // 0 means "not mentioned" — also fine
  };

  size_t stop_pos  = count_type(stop_interfaces,  hardware_interface::HW_IF_POSITION);
  size_t start_pos = count_type(start_interfaces, hardware_interface::HW_IF_POSITION);
  size_t stop_vel  = count_type(stop_interfaces,  hardware_interface::HW_IF_VELOCITY);
  size_t start_vel = count_type(start_interfaces, hardware_interface::HW_IF_VELOCITY);
  size_t stop_eff  = count_type(stop_interfaces,  hardware_interface::HW_IF_EFFORT);
  size_t start_eff = count_type(start_interfaces, hardware_interface::HW_IF_EFFORT);

  if (!validate(stop_pos,  next_position_claimed, false) ||
      !validate(start_pos, next_position_claimed, true)  ||
      !validate(stop_vel,  next_velocity_claimed, false) ||
      !validate(start_vel, next_velocity_claimed, true)  ||
      !validate(stop_eff,  next_effort_claimed,   false) ||
      !validate(start_eff, next_effort_claimed,   true))
  {
    RCLCPP_ERROR(
      getLogger(),
      "prepare_command_mode_switch: partial interface count (pos:%zu/%zu vel:%zu/%zu eff:%zu/%zu)",
      stop_pos, start_pos, stop_vel, start_vel, stop_eff, start_eff);
    return hardware_interface::return_type::ERROR;
  }

  // [Issue: single active mode] At most one command mode may be claimed at a
  // time — the hardware has a single active ActiveControl handle. Reject any
  // transition that would leave two modes simultaneously claimed.
  const int modes_claimed =
    (next_position_claimed ? 1 : 0) +
    (next_velocity_claimed ? 1 : 0) +
    (next_effort_claimed ? 1 : 0);
  if (modes_claimed > 1)
  {
    RCLCPP_ERROR(
      getLogger(),
      "prepare_command_mode_switch: only one command mode may be active at a time "
      "(pos=%d vel=%d eff=%d). Stop the current mode before starting another.",
      next_position_claimed, next_velocity_claimed, next_effort_claimed);
    return hardware_interface::return_type::ERROR;
  }

  position_interface_claimed_ = next_position_claimed;
  velocity_interface_claimed_ = next_velocity_claimed;
  effort_interface_claimed_ = next_effort_claimed;

  return hardware_interface::return_type::OK;
}

// ── perform_command_mode_switch ───────────────────────────────────────────────

hardware_interface::return_type PiperHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  std::lock_guard<std::mutex> lock(mode_switch_mutex_);

  enum class ControlMode { kNone, kPosition, kVelocity, kEffort };

  ControlMode desired = ControlMode::kNone;
  if (effort_interface_claimed_)
  {
    desired = ControlMode::kEffort;
  }
  else if (velocity_interface_claimed_)
  {
    desired = ControlMode::kVelocity;
  }
  else if (position_interface_claimed_)
  {
    desired = ControlMode::kPosition;
  }

  ControlMode current = ControlMode::kNone;
  if (effort_interface_running_.load())
  {
    current = ControlMode::kEffort;
  }
  else if (velocity_interface_running_.load())
  {
    current = ControlMode::kVelocity;
  }
  else if (position_interface_running_.load())
  {
    current = ControlMode::kPosition;
  }

  const bool handle_matches_mode =
    desired == ControlMode::kNone ? !active_control_ : static_cast<bool>(active_control_);
  if (desired == current && handle_matches_mode)
  {
    return hardware_interface::return_type::OK;
  }

  // Match Franka's switch ordering: expose no running mode, stop the old
  // control exactly once, then start only the requested mode.
  position_interface_running_.store(false);
  velocity_interface_running_.store(false);
  effort_interface_running_.store(false);
  stopRobotSafe();

  try
  {
    switch (desired)
    {
      case ControlMode::kEffort:
        std::fill(hw_effort_commands_.begin(), hw_effort_commands_.end(), 0.0);
        active_control_ = robot_->startTorqueControl();
        first_effort_write_.store(true);
        effort_interface_running_.store(true);
        break;

      case ControlMode::kVelocity:
        std::fill(hw_velocity_commands_.begin(), hw_velocity_commands_.end(), 0.0);
        active_control_ = robot_->startJointVelocityControl();
        first_velocity_write_.store(true);
        velocity_interface_running_.store(true);
        break;

      case ControlMode::kPosition:
        // Pre-seed with the last known position; read() will re-seed after the
        // first successful state update (first_position_write_ guards write()).
        std::copy(hw_positions_.begin(), hw_positions_.end(), hw_position_commands_.begin());
        active_control_ =
          robot_->startJointPositionControl(piper::ControlType::kInternalJointPos);
        first_position_write_.store(true);
        position_interface_running_.store(true);
        break;

      case ControlMode::kNone:
        break;
    }
  }
  catch (const std::exception & ex)
  {
    stopRobotSafe();
    RCLCPP_ERROR(getLogger(), "perform_command_mode_switch failed: %s", ex.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

// ── read ──────────────────────────────────────────────────────────────────────

hardware_interface::return_type PiperHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // [Concurrency fix] Protect robot_ against concurrent reset by on_error()/
  // on_cleanup() (CM service thread). Mirrors the try_lock pattern in write().
  // If a cleanup is in progress we skip one read cycle — controllers see stale
  // state for one tick, which is acceptable and far safer than a dangling access.
  std::unique_lock<std::mutex> lock(mode_switch_mutex_, std::try_to_lock);
  if (!lock.owns_lock())
  {
    const uint32_t n = ++read_skip_count_;
    // Log on first skip and every ~500 skips (~1 s at 500 Hz) to avoid flooding.
    if (n == 1 || n % 500 == 0)
    {
      RCLCPP_WARN(
        getLogger(),
        "read(): mode-switch/cleanup in progress, skipped %u cycle(s)", n);
    }
    return hardware_interface::return_type::OK;
  }
  read_skip_count_.store(0);

  if (!robot_)
  {
    return hardware_interface::return_type::ERROR;
  }

  // libpiper readLatest() is non-blocking and preserves the CAN
  // feedback identity across repeated cache reads.
  try
  {
    const auto st = robot_->readLatest();

    const auto now = std::chrono::steady_clock::now();
    const bool timed_out = !st.valid ||
      st.feedback_receive_time == std::chrono::steady_clock::time_point{} ||
      now - st.feedback_receive_time > kFeedbackTimeout;
    if (timed_out)
    {
      RCLCPP_ERROR(
        getLogger(),
        "Piper joint feedback stale/invalid (valid=%d sequence=%llu timeout=%lld ms)",
        st.valid, static_cast<unsigned long long>(st.feedback_sequence),
        static_cast<long long>(kFeedbackTimeout.count()));
      return hardware_interface::return_type::ERROR;
    }
    if (!st.coherent)
    {
      if (!incoherent_feedback_logged_)
      {
        RCLCPP_WARN(
          getLogger(), "Ignoring incoherent Piper joint snapshot sequence=%llu span=%.3f ms",
          static_cast<unsigned long long>(st.feedback_sequence), st.feedback_span.toMSec());
        incoherent_feedback_logged_ = true;
      }
      return hardware_interface::return_type::OK;
    }
    incoherent_feedback_logged_ = false;
    if (st.device_status == piper::FeedbackDeviceStatus::kError)
    {
      RCLCPP_ERROR(
        getLogger(),
        "Piper device reported an error at feedback sequence=%llu",
        static_cast<unsigned long long>(st.feedback_sequence));
      return hardware_interface::return_type::ERROR;
    }

    rt_robot_state_buffer_.writeFromNonRT(st);

    for (size_t i = 0; i < static_cast<size_t>(piper::kNumJoints); ++i)
    {
      hw_positions_[i]  = st.q[i];
      hw_velocities_[i] = st.dq[i];
      hw_efforts_[i]    = st.tau_J[i];
    }

    cartesian_pose_state_ = st.O_T_EE;
    robot_time_state_     = st.time.toSec();

    // First-cycle seeding: once we have real state data, initialise the command
    // buffers to the current measured values and clear the first-write guard.
    if (first_position_write_.load() && position_interface_running_.load())
    {
      for (size_t i = 0; i < piper::kNumJoints; ++i)
      {
        hw_position_commands_[i] = st.q[i];
      }
      first_position_write_.store(false);
    }
    if (first_velocity_write_.load() && velocity_interface_running_.load())
    {
      std::fill(hw_velocity_commands_.begin(), hw_velocity_commands_.end(), 0.0);
      first_velocity_write_.store(false);
    }
    if (first_effort_write_.load() && effort_interface_running_.load())
    {
      std::fill(hw_effort_commands_.begin(), hw_effort_commands_.end(), 0.0);
      first_effort_write_.store(false);
    }
  }
  catch (const std::exception & ex)
  {
    RCLCPP_WARN(getLogger(), "read() failed: %s", ex.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

// ── write ─────────────────────────────────────────────────────────────────────

hardware_interface::return_type PiperHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // [Concurrency fix] Protect active_control_ against concurrent reset by
  // perform_command_mode_switch (CM service thread). Use try_lock so the
  // real-time thread never blocks: if a mode switch is in progress we skip
  // one write cycle, which is safe (libpiper's 500 Hz internal loop will
  // keep sending the last valid command via its latest_command_ buffer).
  std::unique_lock<std::mutex> lock(mode_switch_mutex_, std::try_to_lock);
  if (!lock.owns_lock())
  {
    const uint32_t n = ++write_skip_count_;
    if (n == 1 || n % 500 == 0)
    {
      RCLCPP_WARN(
        getLogger(),
        "write(): mode-switch in progress, skipped %u cycle(s) — "
        "libpiper internal loop holds last command", n);
    }
    return hardware_interface::return_type::OK;  // mode switch in flight, skip
  }
  write_skip_count_.store(0);

  if (!active_control_)
  {
    return hardware_interface::return_type::OK;
  }

  // [Issue 3] Guard against NaN/Inf in all command buffers before touching HW.
  if (hasInfinite(hw_position_commands_) ||
      hasInfinite(hw_velocity_commands_) ||
      hasInfinite(hw_effort_commands_))
  {
    RCLCPP_ERROR(getLogger(), "write(): non-finite value in command buffer — skipping");
    return hardware_interface::return_type::ERROR;
  }

  // active_control_ and mode flags are now safe to access (lock held).
  try
  {
    if (velocity_interface_running_.load())
    {
      if (first_velocity_write_.load()) { return hardware_interface::return_type::OK; }
      std::array<double, piper::kNumJoints> cmd{};
      std::copy(hw_velocity_commands_.begin(), hw_velocity_commands_.end(), cmd.begin());
      active_control_->writeOnce(piper::JointVelocities(cmd));
    }
    else if (effort_interface_running_.load())
    {
      if (first_effort_write_.load()) { return hardware_interface::return_type::OK; }
      std::array<double, piper::kNumJoints> cmd{};
      std::copy(hw_effort_commands_.begin(), hw_effort_commands_.end(), cmd.begin());
      active_control_->writeOnce(piper::Torques(cmd));
    }
    else if (position_interface_running_.load())
    {
      if (first_position_write_.load()) { return hardware_interface::return_type::OK; }
      std::array<double, piper::kNumJoints> cmd{};
      std::copy(hw_position_commands_.begin(), hw_position_commands_.end(), cmd.begin());
      active_control_->writeOnce(piper::JointPositions(cmd));
    }
  }
  catch (const std::exception & ex)
  {
    RCLCPP_WARN(getLogger(), "write() failed: %s", ex.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

// ── stopRobotSafe ─────────────────────────────────────────────────────────────

void PiperHardwareInterface::stopRobotSafe() noexcept
{
  const bool had_active_control = static_cast<bool>(active_control_);

  // ActiveControl's destructor calls Robot::Impl::stop(). Do not follow it
  // with robot_->stop(), because libpiper's stop performs another hold sequence.
  try
  {
    active_control_.reset();
  }
  catch (const std::exception & ex)
  {
    RCLCPP_WARN(getLogger(), "active_control_.reset() threw: %s", ex.what());
  }

  if (robot_ && !had_active_control)
  {
    try
    {
      robot_->stop();
    }
    catch (const std::exception & ex)
    {
      RCLCPP_WARN(getLogger(), "robot_->stop() threw: %s", ex.what());
    }
  }
}

}  // namespace piper_hardware_interface

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
PLUGINLIB_EXPORT_CLASS(
    piper_hardware_interface::PiperHardwareInterface,
    hardware_interface::SystemInterface)
