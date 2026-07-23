#include "mycobot_hardware/mycobot_system.hpp"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <poll.h>
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

namespace mycobot_hardware
{

namespace
{
rclcpp::Logger logger() { return rclcpp::get_logger("mycobot_hardware"); }

// Minimal, dependency-free parse of `"key":[n1,n2,...]` out of a JSON reply.
// Deliberately not pulling in a JSON library for a fixed-shape, hand-built
// protocol between mycobot_system.cpp and mycobot_bridge.py -- see the
// header comment on send_request().
std::vector<double> extract_array(const std::string & json, const std::string & key)
{
  std::vector<double> values;
  auto key_pos = json.find("\"" + key + "\"");
  if (key_pos == std::string::npos) {
    return values;
  }
  auto open = json.find('[', key_pos);
  auto close = json.find(']', open);
  if (open == std::string::npos || close == std::string::npos) {
    return values;
  }
  std::string body = json.substr(open + 1, close - open - 1);
  std::stringstream ss(body);
  std::string token;
  while (std::getline(ss, token, ',')) {
    if (!token.empty()) {
      values.push_back(std::stod(token));
    }
  }
  return values;
}
}  // namespace

MyCobotSystem::CallbackReturn MyCobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const size_t n = info_.joints.size();
  joint_names_.reserve(n);
  position_states_.assign(n, 0.0);
  velocity_states_.assign(n, 0.0);
  effort_states_.assign(n, 0.0);
  position_commands_.assign(n, 0.0);

  for (const auto & joint : info_.joints) {
    joint_names_.push_back(joint.name);
  }

  // Optional <param name="socket_path">...</param> in the ros2_control tag;
  // defaults to a fixed path both this plugin and mycobot_bridge.py agree on.
  auto it = info_.hardware_parameters.find("socket_path");
  socket_path_ = (it != info_.hardware_parameters.end())
    ? it->second
    : "/tmp/mycobot_hardware_bridge.sock";

  RCLCPP_INFO(logger(), "Configured for %zu joints, bridge socket: %s",
              n, socket_path_.c_str());
  return CallbackReturn::SUCCESS;
}

bool MyCobotSystem::has_effort_state(size_t joint_index) const
{
  return joint_index < info_.joints.size() &&
    std::any_of(
      info_.joints[joint_index].state_interfaces.begin(),
      info_.joints[joint_index].state_interfaces.end(),
      [](const auto & iface) { return iface.name == hardware_interface::HW_IF_EFFORT; });
}

std::vector<hardware_interface::StateInterface> MyCobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &position_states_[i]);
    interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]);
    if (has_effort_state(i)) {
      interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT, &effort_states_[i]);
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> MyCobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &position_commands_[i]);
  }
  return interfaces;
}

MyCobotSystem::CallbackReturn MyCobotSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!connect_bridge()) {
    RCLCPP_ERROR(logger(), "Could not connect to mycobot_bridge.py on %s -- "
                 "is it running? (see real_robot.launch.py)", socket_path_.c_str());
    return CallbackReturn::ERROR;
  }

  // Seed command targets from the arm's actual current position so the
  // first write() doesn't command a jump from whatever position the arm is
  // physically holding right now.
  std::string reply = send_request(R"({"cmd":"read"})");
  auto positions = extract_array(reply, "positions");
  if (positions.size() == joint_names_.size()) {
    position_states_ = positions;
    position_commands_ = positions;
  } else {
    RCLCPP_WARN(logger(), "Could not read initial joint state from bridge; "
                "commands will start from 0.0 for all joints.");
  }

  RCLCPP_INFO(logger(), "mycobot hardware activated.");
  return CallbackReturn::SUCCESS;
}

MyCobotSystem::CallbackReturn MyCobotSystem::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  disconnect_bridge();
  RCLCPP_INFO(logger(), "mycobot hardware deactivated.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type MyCobotSystem::read()
{
  std::string reply = send_request(R"({"cmd":"read"})");
  if (reply.empty()) {
    // Bridge unreachable or timed out this cycle -- keep last-known state
    // rather than stall/crash the control loop. Persistent failures show up
    // as the arm silently not tracking; check mycobot_bridge.py's log.
    return hardware_interface::return_type::OK;
  }

  auto positions = extract_array(reply, "positions");
  auto velocities = extract_array(reply, "velocities");
  auto efforts = extract_array(reply, "efforts");

  if (positions.size() == joint_names_.size()) position_states_ = positions;
  if (velocities.size() == joint_names_.size()) velocity_states_ = velocities;
  if (efforts.size() == joint_names_.size()) effort_states_ = efforts;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyCobotSystem::write()
{
  std::ostringstream req;
  req << R"({"cmd":"write","positions":[)";
  for (size_t i = 0; i < position_commands_.size(); ++i) {
    if (i > 0) req << ",";
    req << position_commands_[i];
  }
  req << "]}";

  send_request(req.str());
  return hardware_interface::return_type::OK;
}

bool MyCobotSystem::connect_bridge()
{
  socket_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
  if (socket_fd_ < 0) {
    return false;
  }

  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);

  if (connect(socket_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }
  return true;
}

void MyCobotSystem::disconnect_bridge()
{
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

std::string MyCobotSystem::send_request(const std::string & request, int timeout_ms)
{
  if (socket_fd_ < 0) {
    return "";
  }

  std::string line = request + "\n";
  if (::send(socket_fd_, line.c_str(), line.size(), 0) < 0) {
    RCLCPP_WARN_THROTTLE(logger(), clock_, 5000,
                         "send() to mycobot_bridge.py failed: %s", std::strerror(errno));
    return "";
  }

  pollfd pfd{socket_fd_, POLLIN, 0};
  int ready = poll(&pfd, 1, timeout_ms);
  if (ready <= 0) {
    RCLCPP_WARN_THROTTLE(logger(), clock_, 5000,
                         "mycobot_bridge.py did not reply within %d ms", timeout_ms);
    return "";
  }

  char buf[4096];
  ssize_t n = recv(socket_fd_, buf, sizeof(buf) - 1, 0);
  if (n <= 0) {
    return "";
  }
  buf[n] = '\0';
  return std::string(buf);
}

}  // namespace mycobot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mycobot_hardware::MyCobotSystem, hardware_interface::SystemInterface)
