#include "parol6_hardware_interface/parol6_system_interface.hpp"

#include <cmath>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace parol6_hardware_interface
{

namespace
{
rclcpp::Logger logger() { return rclcpp::get_logger("Parol6SystemInterface"); }

double get_double_param(
  const hardware_interface::HardwareInfo & info, const std::string & name, double fallback)
{
  const auto it = info.hardware_parameters.find(name);
  if (it == info.hardware_parameters.end()) {
    return fallback;
  }
  try {
    return std::stod(it->second);
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger(), "hardware param '%s' = '%s' is not a number (%s); using default %f",
      name.c_str(), it->second.c_str(), e.what(), fallback);
    return fallback;
  }
}

std::string get_string_param(
  const hardware_interface::HardwareInfo & info, const std::string & name,
  const std::string & fallback)
{
  const auto it = info.hardware_parameters.find(name);
  return it == info.hardware_parameters.end() ? fallback : it->second;
}
}  // namespace

hardware_interface::CallbackReturn Parol6SystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kNumJoints) {
    RCLCPP_FATAL(
      logger(), "Parol6SystemInterface expects exactly %zu joints, got %zu", kNumJoints,
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        logger(), "Joint '%s' must expose exactly one command interface: position",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(
        logger(), "Joint '%s' must expose two state interfaces: position, velocity",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  bridge_host_ = get_string_param(info_, "bridge_host", "127.0.0.1");
  bridge_port_ = static_cast<int>(get_double_param(info_, "bridge_port", 6001));
  connect_timeout_sec_ = get_double_param(info_, "connect_timeout_sec", 5.0);

  hw_positions_.assign(kNumJoints, std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.assign(kNumJoints, std::numeric_limits<double>::quiet_NaN());
  hw_commands_position_.assign(kNumJoints, std::numeric_limits<double>::quiet_NaN());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Parol6SystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(kNumJoints * 2);
  for (size_t i = 0; i < kNumJoints; ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> Parol6SystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(kNumJoints);
  for (size_t i = 0; i < kNumJoints; ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn Parol6SystemInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    logger(), "Connecting to parol6_bridge at %s:%d (timeout %.1fs)...", bridge_host_.c_str(),
    bridge_port_, connect_timeout_sec_);

  if (!bridge_.connect(bridge_host_, bridge_port_, connect_timeout_sec_)) {
    RCLCPP_FATAL(
      logger(), "Could not connect to parol6_bridge: %s. Is it running (ros2 run parol6_bridge "
                "parol6_bridge)?",
      bridge_.last_error().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Seed state so the first read()/write() cycle doesn't see NaN / a large jump.
  BridgeResponse resp{};
  std::array<double, 6> zero{};
  if (bridge_.exchange(kOpPing, zero, resp) && resp.ok) {
    for (size_t i = 0; i < kNumJoints; ++i) {
      hw_positions_[i] = resp.pos_rad[i];
      hw_velocities_[i] = resp.vel_rad[i];
      hw_commands_position_[i] = resp.pos_rad[i];
    }
  } else {
    RCLCPP_WARN(logger(), "Initial PING to parol6_bridge did not return a valid state yet");
  }

  RCLCPP_INFO(logger(), "Connected to parol6_bridge");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Parol6SystemInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  BridgeResponse resp{};
  std::array<double, 6> zero{};
  if (!bridge_.exchange(kOpEnable, zero, resp) || !resp.ok) {
    RCLCPP_FATAL(logger(), "Failed to enable robot via parol6_bridge (resume() failed)");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Re-seed the command with current position so activation doesn't jerk
  // the arm toward a stale commanded value.
  for (size_t i = 0; i < kNumJoints; ++i) {
    hw_commands_position_[i] = hw_positions_[i];
  }

  estop_latched_ = false;
  RCLCPP_INFO(logger(), "Robot activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Parol6SystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  BridgeResponse resp{};
  std::array<double, 6> zero{};
  if (!bridge_.exchange(kOpDisable, zero, resp)) {
    RCLCPP_WARN(logger(), "Failed to send halt() to parol6_bridge during deactivate");
  }
  RCLCPP_INFO(logger(), "Robot deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Parol6SystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  BridgeResponse resp{};
  std::array<double, 6> zero{};
  if (!bridge_.exchange(kOpPing, zero, resp) || !resp.ok) {
    RCLCPP_WARN_THROTTLE(
      logger(), clock_, 1000, "parol6_bridge read failed: %s",
      bridge_.last_error().c_str());
    // Keep last known state rather than erroring the whole controller
    // manager out on a transient hiccup.
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < kNumJoints; ++i) {
    hw_positions_[i] = resp.pos_rad[i];
    hw_velocities_[i] = resp.vel_rad[i];
  }

  if (resp.estop && !estop_latched_) {
    RCLCPP_ERROR(logger(), "E-stop is engaged -- motion commands will be rejected by the robot");
    estop_latched_ = true;
  } else if (!resp.estop && estop_latched_) {
    RCLCPP_INFO(logger(), "E-stop cleared");
    estop_latched_ = false;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Parol6SystemInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::array<double, 6> target{};
  for (size_t i = 0; i < kNumJoints; ++i) {
    const double cmd = hw_commands_position_[i];
    target[i] = std::isfinite(cmd) ? cmd : hw_positions_[i];
  }

  BridgeResponse resp{};
  if (!bridge_.exchange(kOpUpdate, target, resp) || !resp.ok) {
    RCLCPP_WARN_THROTTLE(
      logger(), clock_, 1000, "parol6_bridge write failed: %s",
      bridge_.last_error().c_str());
    return hardware_interface::return_type::OK;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace parol6_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  parol6_hardware_interface::Parol6SystemInterface, hardware_interface::SystemInterface)
