#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <servo_driver/srv/head_jog.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

using Clock = std::chrono::steady_clock;
using Bytes = std::vector<uint8_t>;

class ServoDriver : public rclcpp::Node {
  struct State {
    int position = 0, torque = 0, origin = 0, target = 0;
    bool valid = false, pending = false;
    Clock::time_point stamp{}, sent{};
  };
  int fd_ = -1;
  bool allow_motion_, adjusting_ = false;
  static constexpr int arrival_tolerance_ticks_ = 5;
  std::string fault_;
  std::array<State, 2> state_;
  const std::array<int, 2> lower_{1000, 2000}, upper_{2700, 5000};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_;
  rclcpp::Service<servo_driver::srv::HeadJog>::SharedPtr jog_;
  rclcpp::TimerBase::SharedPtr timer_;

  static double age(Clock::time_point stamp) {
    return std::chrono::duration<double>(Clock::now() - stamp).count();
  }
  void lock(const std::string & reason) { adjusting_ = false; fault_ = reason; }
  Bytes exchange(int id, int instruction, const Bytes & params, size_t count) {
    Bytes packet{255, 255, uint8_t(id), uint8_t(params.size() + 2), uint8_t(instruction)};
    packet.insert(packet.end(), params.begin(), params.end());
    unsigned sum = 0;
    for (size_t i = 2; i < packet.size(); ++i) sum += packet[i];
    packet.push_back(uint8_t(~sum));
    tcflush(fd_, TCIFLUSH);
    auto deadline = Clock::now() + std::chrono::milliseconds(80);
    size_t offset = 0;
    while (offset < packet.size()) {
      if (Clock::now() >= deadline) throw std::runtime_error("serial write timeout");
      pollfd p{fd_, POLLOUT, 0};
      if (poll(&p, 1, 5) > 0) {
        auto n = write(fd_, packet.data() + offset, packet.size() - offset);
        if (n > 0) offset += size_t(n);
      }
    }
    Bytes reply;
    while (reply.size() < count + 6 && Clock::now() < deadline) {
      pollfd p{fd_, POLLIN, 0};
      if (poll(&p, 1, 5) > 0) {
        uint8_t b;
        if (read(fd_, &b, 1) == 1) reply.push_back(b);
      }
    }
    if (reply.size() != count + 6) throw std::runtime_error("servo reply timeout");
    sum = 0;
    for (size_t i = 2; i < reply.size(); ++i) sum += reply[i];
    if (reply[0] != 255 || reply[1] != 255 || reply[2] != id ||
        reply[3] != count + 2 || (sum & 255) != 255 || reply[4] != 0)
      throw std::runtime_error("invalid servo reply/checksum or device error");
    return Bytes(reply.begin() + 5, reply.end() - 1);
  }
  void goal(int id, int position, int speed) {
    exchange(id, 3, {42, uint8_t(position), uint8_t(position >> 8), 0, 0,
                    uint8_t(speed), uint8_t(speed >> 8)}, 0);
  }
  void refresh() {
    for (int i = 0; i < 2; ++i) {
      auto & s = state_[i];
      try {
        auto p = exchange(i + 1, 2, {56, 2}, 2);
        auto t = exchange(i + 1, 2, {40, 1}, 1);
        s.position = p[0] | (int(p[1]) << 8);
        s.torque = t[0]; s.valid = true; s.stamp = Clock::now();
        if (s.pending && std::abs(s.position - s.target) <= arrival_tolerance_ticks_) s.pending = false;
        if (s.pending && age(s.sent) > 5.0)
          lock("ID " + std::to_string(i + 1) + " motion timeout; current=" +
               std::to_string(s.position) + " target=" + std::to_string(s.target) +
               "; inspect head before retry");
        if (adjusting_ && s.torque != 1) lock("torque no longer enabled");
      } catch (const std::exception & e) {
        s.valid = false; lock("ID " + std::to_string(i + 1) + ": " + e.what());
      }
    }
  }
  bool ready() const {
    for (const auto & s : state_) if (!s.valid || age(s.stamp) > 0.3) return false;
    return true;
  }
  void publish() {
    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = now();
    for (int i = 0; i < 2; ++i) {
      const auto & s = state_[i];
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = i == 0 ? "head/pitch" : "head/yaw";
      status.hardware_id = std::to_string(i + 1);
      status.level = (!s.valid || !fault_.empty()) ? 2 : 0;
      status.message = fault_.empty() ? (adjusting_ ? "adjustment enabled" : "adjustment locked") : fault_;
      auto add = [&](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue kv; kv.key = key; kv.value = value;
        status.values.push_back(kv);
      };
      add("valid", s.valid && age(s.stamp) <= 0.3 ? "true" : "false");
      add("position_ticks", std::to_string(s.position));
      add("torque_enabled", std::to_string(s.torque));
      add("pending", s.pending ? "true" : "false");
      add("arrival_tolerance_ticks", std::to_string(arrival_tolerance_ticks_));
      add("pending_target_ticks", s.pending ? std::to_string(s.target) : "none");
      add("pending_error_ticks", s.pending ? std::to_string(s.target - s.position) : "none");
      add("pending_elapsed_s", s.pending ? std::to_string(age(s.sent)) : "0");
      add("age_s", std::to_string(age(s.stamp)));
      add("allow_motion", allow_motion_ ? "true" : "false");
      add("adjustment_enabled", adjusting_ ? "true" : "false");
      msg.status.push_back(status);
    }
    pub_->publish(msg);
  }

public:
  ServoDriver() : Node("servo_driver_node") {
    auto port = declare_parameter<std::string>("port", "/dev/serial/by-path/pci-0000:00:14.0-usb-0:5.2:1.0");
    allow_motion_ = declare_parameter<bool>("allow_motion", false);
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    try {
      if (fd_ < 0 || flock(fd_, LOCK_EX | LOCK_NB) != 0 || ioctl(fd_, TIOCEXCL) != 0)
        throw std::runtime_error("cannot exclusively open " + port);
      termios tty{};
      if (tcgetattr(fd_, &tty)) throw std::runtime_error("tcgetattr failed");
      cfmakeraw(&tty); cfsetispeed(&tty, B115200); cfsetospeed(&tty, B115200);
      tty.c_cflag = (tty.c_cflag & ~(PARENB | CSTOPB | CSIZE | CRTSCTS)) | CS8 | CLOCAL | CREAD;
      if (tcsetattr(fd_, TCSANOW, &tty)) throw std::runtime_error("tcsetattr failed");
    } catch (...) { if (fd_ >= 0) close(fd_); fd_ = -1; throw; }
    pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/head/state", 10);
    enable_ = create_service<std_srvs::srv::SetBool>("/head/adjust_enable",
      [this](std_srvs::srv::SetBool::Request::SharedPtr req,
             std_srvs::srv::SetBool::Response::SharedPtr res) {
        try {
          if (!req->data) {
            adjusting_ = false; res->success = true;
            res->message = "adjustment locked; torque and accepted target unchanged"; return;
          }
          if (!allow_motion_) throw std::runtime_error("read-only launch: allow_motion=false");
          refresh();
          if (!ready()) throw std::runtime_error("fresh feedback required for both axes");
          if (adjusting_) { res->success = true; res->message = "already enabled"; return; }
          for (int i = 0; i < 2; ++i) {
            auto & s = state_[i];
            if (s.pending)
              throw std::runtime_error("ID " + std::to_string(i + 1) +
                " pending motion: current=" + std::to_string(s.position) +
                " target=" + std::to_string(s.target) + " error=" +
                std::to_string(s.target - s.position) + " ticks; wait for arrival or inspect fault");
            if (s.position < lower_[i] || s.position > upper_[i])
              throw std::runtime_error("ID " + std::to_string(i + 1) +
                " position=" + std::to_string(s.position) + " outside limits [" +
                std::to_string(lower_[i]) + ", " + std::to_string(upper_[i]) + "]");
          }
          // Overwrite stale controller targets at measured positions before torque engagement.
          for (int i = 0; i < 2; ++i) {
            state_[i].origin = state_[i].position;
            goal(i + 1, state_[i].position, 5);
          }
          for (int i = 0; i < 2; ++i)
            if (state_[i].torque != 1) exchange(i + 1, 3, {40, 1}, 0);
          refresh();
          if (!ready()) throw std::runtime_error("enable readback missing; torque may be enabled");
          for (const auto & s : state_)
            if (s.torque != 1 || std::abs(s.position - s.origin) > 20)
              throw std::runtime_error("enable readback mismatch; inspect head");
          fault_.clear(); adjusting_ = true; res->success = true;
          res->message = "adjustment enabled at measured pose; one small jog at a time";
        } catch (const std::exception & e) { lock(e.what()); res->message = fault_; }
      });
    jog_ = create_service<servo_driver::srv::HeadJog>("/head/jog",
      [this](servo_driver::srv::HeadJog::Request::SharedPtr req,
             servo_driver::srv::HeadJog::Response::SharedPtr res) {
        try {
          if (!allow_motion_ || !adjusting_) throw std::runtime_error("adjustment locked");
          if (req->servo_id < 1 || req->servo_id > 2 || req->delta_ticks == 0 ||
              std::abs(int(req->delta_ticks)) > 20 || req->speed < 1 || req->speed > 5)
            throw std::runtime_error("require ID 1/2, delta +/-1..20 ticks, speed 1..5");
          refresh();
          if (!ready() || !adjusting_) throw std::runtime_error("fresh enabled feedback required");
          for (const auto & s : state_)
            if (s.pending) throw std::runtime_error("previous jog not complete; no queued targets");
          int i = req->servo_id - 1;
          auto & s = state_[i];
          int target = s.position + req->delta_ticks;
          if (target < lower_[i] || target > upper_[i])
            throw std::runtime_error("position outside joint limits");
          s.target = target; s.sent = Clock::now(); s.pending = true;
          goal(req->servo_id, target, req->speed);
          res->success = true; res->message = "accepted target=" + std::to_string(target) + "; await feedback";
        } catch (const std::exception & e) { lock(e.what()); res->message = fault_; }
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { refresh(); publish(); });
    RCLCPP_INFO(get_logger(), "Head startup is read-only. No target or torque command sent. allow_motion=%s",
                allow_motion_ ? "true" : "false");
  }
  ~ServoDriver() override { if (fd_ >= 0) close(fd_); }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<ServoDriver>()); }
  catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("head"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
