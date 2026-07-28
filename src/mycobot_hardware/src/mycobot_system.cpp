#include "mycobot_hardware/mycobot_system.hpp"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <poll.h>
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <sstream>
#include <thread>

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

// Minimal parse of a bare `"key":N` integer field (not inside an array).
// Returns -1 if not found -- fine here since request ids are assigned
// starting at 0 and only ever compared for equality, never used as an
// index, so -1 can never accidentally match a real id.
int extract_int(const std::string & json, const std::string & key)
{
  auto key_pos = json.find("\"" + key + "\"");
  if (key_pos == std::string::npos) {
    return -1;
  }
  auto colon = json.find(':', key_pos);
  if (colon == std::string::npos) {
    return -1;
  }
  try {
    return std::stoi(json.substr(colon + 1));
  } catch (const std::exception &) {
    return -1;
  }
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
  // real_robot.launch.py starts mycobot_bridge.py and ros2_control_node at
  // the same time. The bridge needs real wall-clock time to import
  // pymycobot and open the actual serial connection before its socket
  // exists at all -- a single connect() attempt right after process start
  // reliably loses that race (confirmed: on_activate() failed to connect on
  // every real-hardware test run so far, silently leaving read()/write() as
  // no-ops for the rest of the process's life -- the controller still
  // reported trajectory completion from elapsed time, not real feedback,
  // so nothing physically moved despite "Goal reached" in the logs). Retry
  // for up to ~10s instead of giving up after one attempt.
  const int max_attempts = 50;
  const int retry_delay_ms = 200;

  for (int attempt = 0; attempt < max_attempts; ++attempt) {
    socket_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
    if (socket_fd_ < 0) {
      return false;
    }

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);

    if (connect(socket_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == 0) {
      if (attempt > 0) {
        RCLCPP_INFO(logger(), "Connected to mycobot_bridge.py after %d retr%s",
                    attempt, attempt == 1 ? "y" : "ies");
      }
      return true;
    }

    close(socket_fd_);
    socket_fd_ = -1;
    std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
  }

  return false;
}

void MyCobotSystem::disconnect_bridge()
{
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
  recv_buf_.clear();
}

std::string MyCobotSystem::read_line(std::chrono::steady_clock::time_point deadline)
{
  while (true) {
    auto nl = recv_buf_.find('\n');
    if (nl != std::string::npos) {
      std::string line = recv_buf_.substr(0, nl);
      recv_buf_.erase(0, nl + 1);
      return line;
    }

    auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return "";
    }
    int remaining_ms = static_cast<int>(
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());

    pollfd pfd{socket_fd_, POLLIN, 0};
    int ready = poll(&pfd, 1, remaining_ms);
    if (ready <= 0) {
      return "";
    }

    char buf[4096];
    ssize_t n = recv(socket_fd_, buf, sizeof(buf), 0);
    if (n <= 0) {
      return "";
    }
    recv_buf_.append(buf, static_cast<size_t>(n));
  }
}

std::string MyCobotSystem::send_request(const std::string & request_body, int timeout_ms)
{
  if (socket_fd_ < 0) {
    return "";
  }

  // Splice `,"id":N` into the request just before its closing brace, e.g.
  // `{"cmd":"read"}` -> `{"cmd":"read","id":7}`. request_body is always a
  // hand-built single-line JSON object from a call site in this file, so
  // this string surgery is safe (no nested braces at the top level, no
  // trailing whitespace).
  int id = next_request_id_++;
  auto close_brace = request_body.rfind('}');
  std::string tagged = request_body.substr(0, close_brace) +
    R"(,"id":)" + std::to_string(id) + "}";

  std::string line = tagged + "\n";
  if (::send(socket_fd_, line.c_str(), line.size(), 0) < 0) {
    RCLCPP_WARN_THROTTLE(logger(), clock_, 5000,
                         "send() to mycobot_bridge.py failed: %s", std::strerror(errno));
    return "";
  }

  // Read replies until one matches the id just sent, discarding any stale
  // replies left over from a previous call whose deadline expired before
  // the bridge's answer actually arrived on the wire. Without this, the
  // next call would consume that stale reply as if it were its own,
  // silently mispairing every request/reply from that point on -- this is
  // what caused position_states_/position_commands_ to reflect old,
  // unrelated cycles instead of the current one (see commit message).
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  int discarded = 0;
  while (true) {
    std::string reply = read_line(deadline);
    if (reply.empty()) {
      if (discarded > 0) {
        RCLCPP_WARN_THROTTLE(logger(), clock_, 5000,
                             "mycobot_bridge.py: discarded %d stale repl%s waiting for id %d, "
                             "then timed out", discarded, discarded == 1 ? "y" : "ies", id);
      } else {
        RCLCPP_WARN_THROTTLE(logger(), clock_, 5000,
                             "mycobot_bridge.py did not reply within %d ms", timeout_ms);
      }
      return "";
    }

    if (extract_int(reply, "id") == id) {
      return reply;
    }
    ++discarded;
  }
}

}  // namespace mycobot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mycobot_hardware::MyCobotSystem, hardware_interface::SystemInterface)
