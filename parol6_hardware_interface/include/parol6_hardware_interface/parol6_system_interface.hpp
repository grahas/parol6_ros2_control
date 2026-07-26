#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "parol6_hardware_interface/bridge_client.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace parol6_hardware_interface
{

/// ros2_control SystemInterface for the 6-DOF PAROL6 arm. Reads/writes go
/// to a parol6_bridge daemon over a local TCP socket (see bridge_client.hpp)
/// rather than talking to parol6-server's UDP protocol directly -- the
/// bridge does that using parol6's own Python SDK.
class Parol6SystemInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Parol6SystemInterface)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr size_t kNumJoints = 6;

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_position_;

  std::string bridge_host_{"127.0.0.1"};
  int bridge_port_{6001};
  double connect_timeout_sec_{5.0};

  BridgeClient bridge_;
  bool estop_latched_{false};
  rclcpp::Clock clock_{RCL_STEADY_TIME};
};

}  // namespace parol6_hardware_interface
