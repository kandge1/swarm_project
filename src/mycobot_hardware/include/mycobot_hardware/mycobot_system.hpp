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

  // Every request carries a monotonically increasing id, echoed back by the
  // bridge in its reply. Needed because send_request() can time out and
  // return before the bridge's reply for that request actually arrives on
  // the wire -- without an id, the NEXT send_request() call would read that
  // late reply and mistake it for the answer to a different, newer request,
  // silently desyncing request/reply pairing forever after. See
  // send_request()'s comment for how this is used to self-heal.
  int next_request_id_ = 0;
  // Bytes read from the socket but not yet split into complete lines --
  // persists across send_request() calls since a stale reply drained while
  // looking for one request's answer may contain the start of the next
  // reply too.
  std::string recv_buf_;

  bool connect_bridge();
  void disconnect_bridge();
  // Sends `request` (a JSON object missing its "id" field, no trailing
  // newline -- send_request() adds both) and returns the matching reply
  // line (without trailing newline), or an empty string on failure/timeout.
  // timeout_ms bounds the total time spent waiting, including time spent
  // discarding any stale replies left over from a previous timed-out call.
  std::string send_request(const std::string & request_body, int timeout_ms = 200);
  // Reads and returns one newline-delimited line from the socket, blocking
  // up to `deadline`. Returns empty string on timeout/error. Pulls from
  // recv_buf_ first before touching the socket.
  std::string read_line(std::chrono::steady_clock::time_point deadline);
};

}  // namespace mycobot_hardware

#endif  // MYCOBOT_HARDWARE__MYCOBOT_SYSTEM_HPP_
