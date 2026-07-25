#ifndef MYCOBOT_HARDWARE__MYCOBOT_SYSTEM_HPP_
#define MYCOBOT_HARDWARE__MYCOBOT_SYSTEM_HPP_

#include <chrono>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace mycobot_hardware
{

// ros2_control SystemInterface for the physical mycobot 280 Pi.
//
// Talks to a persistent Python process (scripts/mycobot_bridge.py) over a
// Unix domain socket rather than reimplementing the arm's serial protocol in
// C++: elephantrobotics' pymycobot library is the vendor-tested driver for
// this exact hardware (already used elsewhere in this project, e.g.
// gripper_test.py), and re-deriving that protocol from scratch in C++ would
// be redundant, error-prone, and slower to get right than reusing it.
// read()/write() send a small newline-delimited JSON request per call and
// block briefly for a reply; on timeout the last-known state is kept rather
// than stalling the control loop or crashing.
//
// Signatures below target ros2_control as it exists in ROS2 Galactic
// (confirmed against control.ros.org/galactic API docs): on_init returns
// CallbackReturn; on_activate/on_deactivate take
// const rclcpp_lifecycle::State&; read()/write() take NO time/duration
// arguments (that signature change came in a later ROS2 distro) and return
// hardware_interface::return_type.
class MyCobotSystem : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read() override;
  hardware_interface::return_type write() override;

private:
  // One entry per joint declared in firefighter.ros2_control.xacro, in
  // info_.joints order: the 6 arm joints (position+velocity state, position
  // command) and gripper_controller (position+velocity+effort state,
  // position command). effort_states_ for gripper_controller is a
  // placeholder (see mycobot_bridge.py) -- pymycobot's gripper API exposes
  // no force/effort reading at all, so pick_place.py's
  // gripper_close_until_contact() effort-based contact detection CANNOT
  // work against real hardware as-is. That needs a separate design decision
  // (e.g. is_gripper_moving()-based stall detection instead of effort) --
  // not solved here.
  std::vector<std::string> joint_names_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  std::vector<double> effort_states_;
  std::vector<double> position_commands_;

  bool has_effort_state(size_t joint_index) const;

  // ---- Unix domain socket bridge to mycobot_bridge.py ----
  std::string socket_path_;
  int socket_fd_ = -1;
  rclcpp::Clock clock_{RCL_STEADY_TIME};

  // Debug timing only -- measures actual interval between successive
  // read()/write() calls to see if the control loop itself is falling
  // behind its 100Hz nominal rate, vs. the bridge round-trip being slow.
  std::chrono::steady_clock::time_point last_read_time_{};
  std::chrono::steady_clock::time_point last_write_time_{};

  bool connect_bridge();
  void disconnect_bridge();
  // Sends `request` (a raw JSON string, no trailing newline) and returns the
  // reply line (without trailing newline), or an empty string on any
  // failure/timeout. timeout_ms bounds the blocking read.
  std::string send_request(const std::string & request, int timeout_ms = 200);
};

}  // namespace mycobot_hardware

#endif  // MYCOBOT_HARDWARE__MYCOBOT_SYSTEM_HPP_
