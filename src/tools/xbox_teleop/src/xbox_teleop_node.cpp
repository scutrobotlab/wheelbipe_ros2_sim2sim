// Copyright (c) 2025 SCUTRobotLab
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <dirent.h>
#include <fcntl.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>

namespace {

std::string lowerCopy(const std::string& value) {
  std::string result = value;
  std::transform(result.begin(), result.end(), result.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return result;
}

std::string normalizePrefix(std::string prefix) {
  if (prefix.empty()) {
    return prefix;
  }
  if (prefix.front() != '/') {
    prefix = "/" + prefix;
  }
  while (prefix.size() > 1 && prefix.back() == '/') {
    prefix.pop_back();
  }
  return prefix;
}

std::string prefixedTopic(const std::string& prefix, const std::string& topic) {
  if (topic.empty()) {
    return topic;
  }
  if (topic.front() == '/') {
    return topic;
  }
  return prefix.empty() ? topic : prefix + "/" + topic;
}

}  // namespace

class XboxTeleopNode : public rclcpp::Node {
 public:
  XboxTeleopNode() : Node("xbox_teleop_node") {
    declareParameters();
    loadParameters();
    configureRosInterfaces();

    last_publish_time_ = std::chrono::steady_clock::now();
    running_.store(true);
    input_thread_ = std::thread(&XboxTeleopNode::inputLoop, this);

    const auto publish_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_));
    publish_timer_ =
        create_wall_timer(publish_period, std::bind(&XboxTeleopNode::publishCommands, this));
    if (display_status_) {
      const auto status_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / status_rate_hz_));
      status_timer_ =
          create_wall_timer(status_period, std::bind(&XboxTeleopNode::printStatus, this));
    }

    RCLCPP_INFO(get_logger(), "Xbox teleop bridge started. Device: '%s' hint='%s'",
                device_path_.empty() ? "<auto>" : device_path_.c_str(), device_name_hint_.c_str());
  }

  ~XboxTeleopNode() override {
    running_.store(false);
    if (input_thread_.joinable()) {
      input_thread_.join();
    }
    closeDevice();
  }

 private:
  void declareParameters() {
    declare_parameter<std::string>("device_path", "");
    declare_parameter<std::string>("device_name_hint", "Xbox");
    declare_parameter<bool>("grab_device", false);
    declare_parameter<std::string>("prefix", "wheelbipe_V14");
    declare_parameter<std::string>("motion_command_topic", "motion_command");
    declare_parameter<std::string>("height_command_topic", "height_command");
    declare_parameter<std::string>("state_command_topic", "state_command");
    declare_parameter<std::string>("current_state_topic", "current_state");
    declare_parameter<double>("publish_rate_hz", 50.0);
    declare_parameter<int>("device_retry_period_ms", 1000);
    declare_parameter<bool>("display_status", true);
    declare_parameter<bool>("clear_status_screen", true);
    declare_parameter<double>("status_rate_hz", 5.0);
    declare_parameter<bool>("print_raw_events", false);

    declare_parameter<int>("button_start_code", BTN_START);
    declare_parameter<int>("button_record_code", KEY_RECORD);

    declare_parameter<int>("axis_left_y_code", ABS_Y);
    declare_parameter<int>("axis_right_x_code", ABS_RX);
    declare_parameter<int>("axis_left_trigger_code", ABS_Z);
    declare_parameter<int>("axis_right_trigger_code", ABS_RZ);

    declare_parameter<int>("stick_deadzone", 3000);
    declare_parameter<double>("axis_abs_max", 32767.0);
    declare_parameter<double>("trigger_abs_max", 1023.0);
    declare_parameter<double>("trigger_deadzone", 2.0);

    declare_parameter<double>("linear_x_axis_sign", -1.0);
    declare_parameter<double>("angular_z_axis_sign", -1.0);
    declare_parameter<double>("max_linear_x", 2.5);
    declare_parameter<double>("max_angular_z", 3.0);

    declare_parameter<double>("min_height", 0.20);
    declare_parameter<double>("max_height", 0.40);
    declare_parameter<double>("default_height", 0.22);
    declare_parameter<double>("height_down_rate", 0.2);
    declare_parameter<double>("height_up_rate", 0.2);
    declare_parameter<int>("idle_state_id", 1);
    declare_parameter<int>("rl_state_id", 3);
  }

  void loadParameters() {
    device_path_ = get_parameter("device_path").as_string();
    device_name_hint_ = get_parameter("device_name_hint").as_string();
    grab_device_ = get_parameter("grab_device").as_bool();
    prefix_ = normalizePrefix(get_parameter("prefix").as_string());

    motion_topic_ = prefixedTopic(prefix_, get_parameter("motion_command_topic").as_string());
    height_topic_ = prefixedTopic(prefix_, get_parameter("height_command_topic").as_string());
    state_topic_ = prefixedTopic(prefix_, get_parameter("state_command_topic").as_string());
    current_state_topic_ = prefixedTopic(prefix_, get_parameter("current_state_topic").as_string());

    publish_rate_hz_ = std::max(1.0, get_parameter("publish_rate_hz").as_double());
    device_retry_period_ms_ =
        std::max(50, static_cast<int>(get_parameter("device_retry_period_ms").as_int()));
    display_status_ = get_parameter("display_status").as_bool();
    clear_status_screen_ = get_parameter("clear_status_screen").as_bool();
    status_rate_hz_ = std::max(0.5, get_parameter("status_rate_hz").as_double());
    print_raw_events_ = get_parameter("print_raw_events").as_bool();

    button_start_code_ = get_parameter("button_start_code").as_int();
    button_record_code_ = get_parameter("button_record_code").as_int();

    axis_left_y_code_ = get_parameter("axis_left_y_code").as_int();
    axis_right_x_code_ = get_parameter("axis_right_x_code").as_int();
    axis_left_trigger_code_ = get_parameter("axis_left_trigger_code").as_int();
    axis_right_trigger_code_ = get_parameter("axis_right_trigger_code").as_int();

    stick_deadzone_ = std::max(0, static_cast<int>(get_parameter("stick_deadzone").as_int()));
    axis_abs_max_ = std::max(1.0, get_parameter("axis_abs_max").as_double());
    trigger_abs_max_ = std::max(1.0, get_parameter("trigger_abs_max").as_double());
    trigger_deadzone_ = std::max(0.0, get_parameter("trigger_deadzone").as_double());

    linear_x_axis_sign_ = get_parameter("linear_x_axis_sign").as_double();
    angular_z_axis_sign_ = get_parameter("angular_z_axis_sign").as_double();
    max_linear_x_ = std::max(0.0, get_parameter("max_linear_x").as_double());
    max_angular_z_ = std::max(0.0, get_parameter("max_angular_z").as_double());

    min_height_ = get_parameter("min_height").as_double();
    max_height_ = get_parameter("max_height").as_double();
    if (min_height_ > max_height_) {
      std::swap(min_height_, max_height_);
    }
    default_height_ =
        std::clamp(get_parameter("default_height").as_double(), min_height_, max_height_);
    current_height_ = default_height_;

    height_down_rate_ = std::max(0.0, get_parameter("height_down_rate").as_double());
    height_up_rate_ = std::max(0.0, get_parameter("height_up_rate").as_double());
    idle_state_id_ = static_cast<int>(get_parameter("idle_state_id").as_int());
    rl_state_id_ = static_cast<int>(get_parameter("rl_state_id").as_int());
  }

  void configureRosInterfaces() {
    auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    command_qos.best_effort();
    command_qos.durability_volatile();

    motion_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(motion_topic_, command_qos);
    height_cmd_pub_ = create_publisher<std_msgs::msg::Float64>(height_topic_, command_qos);
    state_cmd_pub_ = create_publisher<std_msgs::msg::Int32>(state_topic_, command_qos);
    current_state_sub_ = create_subscription<std_msgs::msg::Int32>(
        current_state_topic_, command_qos,
        std::bind(&XboxTeleopNode::currentStateCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "Publishing motion='%s', height='%s', state='%s'. Subscribing current_state='%s'",
                motion_cmd_pub_->get_topic_name(), height_cmd_pub_->get_topic_name(),
                state_cmd_pub_->get_topic_name(), current_state_sub_->get_topic_name());
  }

  void inputLoop() {
    while (rclcpp::ok() && running_.load()) {
      if (device_fd_ < 0 && !openDevice()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(device_retry_period_ms_));
        continue;
      }

      input_event event{};
      const ssize_t bytes = read(device_fd_, &event, sizeof(event));
      if (bytes == static_cast<ssize_t>(sizeof(event))) {
        handleInputEvent(event);
      } else if (bytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
        } else {
          RCLCPP_WARN(get_logger(), "Input read failed on '%s': %s", active_device_path_.c_str(),
                      std::strerror(errno));
          closeDevice();
        }
      } else if (bytes == 0) {
        RCLCPP_WARN(get_logger(), "Xbox input device '%s' reached EOF; treating it as disconnected",
                    active_device_path_.c_str());
        closeDevice();
      } else {
        RCLCPP_WARN(get_logger(), "Partial input_event read from '%s'",
                    active_device_path_.c_str());
        closeDevice();
      }
    }
  }

  bool openDevice() {
    const std::string path = device_path_.empty() ? discoverDevicePath() : device_path_;
    if (path.empty()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "No Xbox evdev device found. Set device_path if auto-discovery misses it.");
      return false;
    }

    const int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Failed to open '%s': %s",
                           path.c_str(), std::strerror(errno));
      return false;
    }

    char name[256]{};
    if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) < 0) {
      std::strncpy(name, "<unknown>", sizeof(name) - 1);
    }

    if (grab_device_) {
      if (ioctl(fd, EVIOCGRAB, 1) < 0) {
        RCLCPP_WARN(get_logger(), "EVIOCGRAB failed for '%s': %s", path.c_str(),
                    std::strerror(errno));
      }
    }

    device_fd_ = fd;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      active_device_path_ = path;
      active_device_name_ = name;
      device_connected_ = true;
      last_action_text_ = "device opened";
    }
    RCLCPP_INFO(get_logger(), "Opened Xbox input device '%s' (%s)", path.c_str(), name);
    return true;
  }

  void closeDevice() {
    if (device_fd_ >= 0) {
      if (grab_device_) {
        ioctl(device_fd_, EVIOCGRAB, 0);
      }
      close(device_fd_);
      device_fd_ = -1;
    }
    bool publish_stop = false;
    double safe_height = default_height_;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      publish_stop = device_connected_;
      safe_height = current_height_;
      current_linear_x_ = 0.0;
      current_angular_z_ = 0.0;
      left_trigger_ = 0.0;
      right_trigger_ = 0.0;
      device_connected_ = false;
      active_device_path_.clear();
      active_device_name_.clear();
      last_action_text_ = "device disconnected: zero motion command";
    }

    if (publish_stop && rclcpp::ok()) {
      publishSafeStop(safe_height);
      RCLCPP_WARN(get_logger(), "Xbox disconnected; published an immediate zero-motion command");
    }
  }

  void publishSafeStop(double height) {
    geometry_msgs::msg::Twist motion_msg;
    motion_cmd_pub_->publish(motion_msg);

    std_msgs::msg::Float64 height_msg;
    height_msg.data = std::isfinite(height) ? height : default_height_;
    height_cmd_pub_->publish(height_msg);
  }

  std::string discoverDevicePath() {
    DIR* dir = opendir("/dev/input");
    if (!dir) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Cannot open /dev/input: %s",
                           std::strerror(errno));
      return "";
    }

    std::vector<std::string> candidates;
    struct dirent* entry = nullptr;
    while ((entry = readdir(dir)) != nullptr) {
      const std::string name(entry->d_name);
      if (name.rfind("event", 0) == 0) {
        candidates.emplace_back("/dev/input/" + name);
      }
    }
    closedir(dir);
    std::sort(candidates.begin(), candidates.end());

    const std::string hint = lowerCopy(device_name_hint_);
    for (const auto& path : candidates) {
      const int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
      if (fd < 0) {
        continue;
      }
      char device_name[256]{};
      // EVIOCGNAME returns the copied name length on success, not necessarily zero.
      const int name_length = ioctl(fd, EVIOCGNAME(sizeof(device_name)), device_name);
      if (name_length >= 0) {
        const std::string lowered_name = lowerCopy(device_name);
        const bool matches_hint = hint.empty() || lowered_name.find(hint) != std::string::npos;
        if (matches_hint) {
          close(fd);
          return path;
        }
      }
      close(fd);
    }
    return "";
  }

  void handleInputEvent(const input_event& event) {
    recordRawEvent(event);
    if (event.type == EV_KEY) {
      handleButtonEvent(static_cast<int>(event.code), static_cast<int>(event.value));
    } else if (event.type == EV_ABS) {
      handleAxisEvent(static_cast<int>(event.code), static_cast<int>(event.value));
    }
  }

  void recordRawEvent(const input_event& event) {
    std::ostringstream stream;
    stream << "type=" << event.type << " code=" << event.code << " value=" << event.value;
    const auto text = stream.str();
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      last_event_text_ = text;
    }
    if (print_raw_events_) {
      RCLCPP_INFO(get_logger(), "evdev %s", text.c_str());
    }
  }

  void handleButtonEvent(int code, int value) {
    if (value != 1) {
      return;
    }

    if (code == button_start_code_) {
      publishStateCommand(rl_state_id_, "RL");
    } else if (code == button_record_code_) {
      publishStateCommand(idle_state_id_, "IDLE");
    }
  }

  void handleAxisEvent(int code, int value) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    if (code == axis_left_y_code_) {
      current_linear_x_ = normalizeStick(value, linear_x_axis_sign_, max_linear_x_);
      last_action_text_ = "ABS_Y -> linear.x";
    } else if (code == axis_right_x_code_) {
      current_angular_z_ = normalizeStick(value, angular_z_axis_sign_, max_angular_z_);
      last_action_text_ = "ABS_RX -> angular.z";
    } else if (code == axis_left_trigger_code_) {
      left_trigger_ = normalizeTrigger(value);
      last_action_text_ = "ABS_Z -> height down velocity";
    } else if (code == axis_right_trigger_code_) {
      right_trigger_ = normalizeTrigger(value);
      last_action_text_ = "ABS_RZ -> height up velocity";
    }
  }

  double normalizeStick(int raw_value, double sign, double max_command) const {
    if (std::abs(raw_value) < stick_deadzone_) {
      return 0.0;
    }
    const double normalized =
        std::clamp(sign * static_cast<double>(raw_value) / axis_abs_max_, -1.0, 1.0);
    return normalized * max_command;
  }

  double normalizeTrigger(int raw_value) const {
    if (static_cast<double>(raw_value) <= trigger_deadzone_) {
      return 0.0;
    }
    return std::clamp(static_cast<double>(raw_value) / trigger_abs_max_, 0.0, 1.0);
  }

  void publishStateCommand(int state_id, const char* label) {
    std_msgs::msg::Int32 msg;
    msg.data = state_id;
    state_cmd_pub_->publish(msg);
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      last_commanded_state_id_ = state_id;
      last_action_text_ =
          std::string("state_command -> ") + std::to_string(state_id) + " (" + label + ")";
    }
    RCLCPP_INFO(get_logger(), "state_command -> %d (%s)", msg.data, label);
  }

  void currentStateCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    display_state_id_ = msg->data;
  }

  const char* stateName(int state_id) const {
    switch (state_id) {
      case 0:
        return "INIT";
      case 1:
        return "IDLE";
      case 2:
        return "PREPARE";
      case 3:
        return "RL";
      default:
        return "?";
    }
  }

  void printStatus() {
    double linear_x = 0.0;
    double angular_z = 0.0;
    double height = 0.0;
    double left_trigger = 0.0;
    double right_trigger = 0.0;
    bool device_connected = false;
    int display_state_id = -1;
    int last_commanded_state_id = -1;
    std::string event_text;
    std::string action_text;
    std::string device_path;
    std::string device_name;

    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      linear_x = current_linear_x_;
      angular_z = current_angular_z_;
      height = current_height_;
      left_trigger = left_trigger_;
      right_trigger = right_trigger_;
      device_connected = device_connected_;
      display_state_id = display_state_id_;
      last_commanded_state_id = last_commanded_state_id_;
      event_text = last_event_text_;
      action_text = last_action_text_;
      device_path = active_device_path_;
      device_name = active_device_name_;
    }

    if (clear_status_screen_) {
      std::cout << "\033[2J\033[H";
    }

    std::cout << "-------------------------------------------------------\n";
    std::cout << "            Xbox Teleoperation Control\n";
    std::cout << "-------------------------------------------------------\n";
    std::cout << "Device: " << (device_connected ? "CONNECTED" : "WAITING");
    if (!device_path.empty()) {
      std::cout << "  " << device_path;
    } else if (!device_path_.empty()) {
      std::cout << "  " << device_path_;
    } else {
      std::cout << "  auto hint='" << device_name_hint_ << "'";
    }
    if (!device_name.empty()) {
      std::cout << "  (" << device_name << ")";
    }
    std::cout << "\n";
    std::cout << "Topics: motion=" << motion_topic_ << " height=" << height_topic_
              << " state=" << state_topic_ << "\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Command: vx=" << std::setw(7) << linear_x << "  yaw=" << std::setw(8) << angular_z
              << "  height=" << std::setw(6) << height << "\n";
    std::cout << "Inputs : LT=" << std::setw(5) << left_trigger << "  RT=" << std::setw(5)
              << right_trigger << "\n";
    std::cout << "State  : controller=";
    if (display_state_id >= 0) {
      std::cout << display_state_id << " (" << stateName(display_state_id) << ")";
    } else {
      std::cout << "-";
    }
    std::cout << "  last_cmd=";
    if (last_commanded_state_id >= 0) {
      std::cout << last_commanded_state_id << " (" << stateName(last_commanded_state_id) << ")";
    } else {
      std::cout << "-";
    }
    std::cout << "\n";
    std::cout << "Last evdev : " << (event_text.empty() ? "-" : event_text) << "\n";
    std::cout << "Last action: " << (action_text.empty() ? "-" : action_text) << "\n";
    std::cout << "Buttons: START RL | SHARE/RECORD IDLE\n";
    std::cout << "Axes   : ABS_Y vx | ABS_RX yaw | ABS_Z height down | ABS_RZ height up\n";
    std::cout << "-------------------------------------------------------\n";
    std::cout.flush();
  }

  void publishCommands() {
    const auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_publish_time_).count();
    last_publish_time_ = now;
    if (!std::isfinite(dt) || dt < 0.0) {
      dt = 0.0;
    }
    dt = std::min(dt, 0.2);

    double linear_x = 0.0;
    double angular_z = 0.0;
    double height = default_height_;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      if (device_connected_) {
        const double height_velocity =
            right_trigger_ * height_up_rate_ - left_trigger_ * height_down_rate_;
        current_height_ =
            std::clamp(current_height_ + height_velocity * dt, min_height_, max_height_);
        linear_x = current_linear_x_;
        angular_z = current_angular_z_;
      } else {
        current_linear_x_ = 0.0;
        current_angular_z_ = 0.0;
        left_trigger_ = 0.0;
        right_trigger_ = 0.0;
      }
      height = current_height_;
    }

    geometry_msgs::msg::Twist motion_msg;
    motion_msg.linear.x = linear_x;
    motion_msg.linear.y = 0.0;
    motion_msg.linear.z = 0.0;
    motion_msg.angular.x = 0.0;
    motion_msg.angular.y = 0.0;
    motion_msg.angular.z = angular_z;
    motion_cmd_pub_->publish(motion_msg);

    std_msgs::msg::Float64 height_msg;
    height_msg.data = height;
    height_cmd_pub_->publish(height_msg);

    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "Xbox cmd vx=%.3f yaw=%.3f height=%.3f",
                          linear_x, angular_z, height);
  }

  std::atomic<bool> running_{false};
  std::thread input_thread_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr height_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr current_state_sub_;

  std::mutex cmd_mutex_;
  std::chrono::steady_clock::time_point last_publish_time_;

  int device_fd_{-1};
  std::string active_device_path_;
  std::string device_path_;
  std::string device_name_hint_;
  bool grab_device_{false};
  std::string prefix_;
  std::string motion_topic_;
  std::string height_topic_;
  std::string state_topic_;
  std::string current_state_topic_;
  double publish_rate_hz_{50.0};
  int device_retry_period_ms_{1000};
  bool display_status_{true};
  bool clear_status_screen_{true};
  double status_rate_hz_{5.0};
  bool print_raw_events_{false};

  int button_start_code_{BTN_START};
  int button_record_code_{KEY_RECORD};
  int axis_left_y_code_{ABS_Y};
  int axis_right_x_code_{ABS_RX};
  int axis_left_trigger_code_{ABS_Z};
  int axis_right_trigger_code_{ABS_RZ};

  int stick_deadzone_{3000};
  double axis_abs_max_{32767.0};
  double trigger_abs_max_{1023.0};
  double trigger_deadzone_{2.0};
  double linear_x_axis_sign_{-1.0};
  double angular_z_axis_sign_{1.0};
  double max_linear_x_{2.5};
  double max_angular_z_{3.0};
  double min_height_{0.20};
  double max_height_{0.40};
  double default_height_{0.22};
  double height_down_rate_{0.2};
  double height_up_rate_{0.2};
  int idle_state_id_{1};
  int rl_state_id_{3};

  double current_linear_x_{0.0};
  double current_angular_z_{0.0};
  double current_height_{0.22};
  double left_trigger_{0.0};
  double right_trigger_{0.0};
  bool device_connected_{false};
  int display_state_id_{-1};
  int last_commanded_state_id_{-1};
  std::string active_device_name_;
  std::string last_event_text_{"-"};
  std::string last_action_text_{"waiting for controller input"};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XboxTeleopNode>());
  rclcpp::shutdown();
  return 0;
}
