#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace
{
speed_t baud_to_termios(int baud)
{
  switch (baud) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    default:
      return B115200;
  }
}
}  // namespace

struct MotorConfig
{
  std::string name;
  int64_t interval_us;
  int64_t direction;
  bool run_on_start;
};

class ExtruderSerialNode : public rclcpp::Node
{
public:
  ExtruderSerialNode()
  : Node("extruder_serial_node")
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyACM0");
    baud_ = declare_parameter<int>("baud", 115200);
    control_mode_ = declare_parameter<std::string>("control_mode", "mixed");
    read_status_on_start_ = declare_parameter<bool>("read_status_on_start", true);

    motors_ = {
      {"A", declare_parameter<int>("motors.a.interval_us", 50),
            declare_parameter<int>("motors.a.direction", 0),
            declare_parameter<bool>("motors.a.run_on_start", false)},
      {"B", declare_parameter<int>("motors.b.interval_us", 3000),
            declare_parameter<int>("motors.b.direction", 0),
            declare_parameter<bool>("motors.b.run_on_start", false)},
      {"C", declare_parameter<int>("motors.c.interval_us", 50),
            declare_parameter<int>("motors.c.direction", 0),
            declare_parameter<bool>("motors.c.run_on_start", false)},
    };

    command_sub_ = create_subscription<std_msgs::msg::String>(
      "extruder/command",
      10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        send_line(msg->data);
      });
    status_pub_ = create_publisher<std_msgs::msg::String>("extruder/status", 10);
    read_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      [this]() {
        read_serial();
      });

    open_serial();
    apply_startup_config();
  }

  ~ExtruderSerialNode() override
  {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

private:
  void open_serial()
  {
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to open %s: %s", port_.c_str(), strerror(errno));
      return;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", strerror(errno));
      close(fd_);
      fd_ = -1;
      return;
    }

    cfmakeraw(&tty);
    const speed_t speed = baud_to_termios(baud_);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", strerror(errno));
      close(fd_);
      fd_ = -1;
      return;
    }

    RCLCPP_INFO(get_logger(), "Opened extruder serial port %s at %d baud", port_.c_str(), baud_);
  }

  bool send_line(const std::string & line)
  {
    if (fd_ < 0) {
      RCLCPP_WARN(get_logger(), "Serial port is not open, dropped command: %s", line.c_str());
      return false;
    }

    std::string payload = line;
    if (payload.empty() || payload.back() != '\n') {
      payload.push_back('\n');
    }

    const ssize_t written = write(fd_, payload.c_str(), payload.size());
    if (written != static_cast<ssize_t>(payload.size())) {
      RCLCPP_ERROR(get_logger(), "Failed to write command '%s': %s", line.c_str(), strerror(errno));
      return false;
    }

    RCLCPP_INFO(get_logger(), "extruder << %s", line.c_str());
    return true;
  }

  void apply_startup_config()
  {
    if (fd_ < 0) {
      return;
    }

    send_line("MODE " + normalize_control_mode(control_mode_));

    for (const auto & motor : motors_) {
      send_line("INTERVAL_US " + motor.name + " " + std::to_string(motor.interval_us));
      send_line("DIR " + motor.name + " " + std::to_string(motor.direction));
      send_line("RUN " + motor.name + " " + std::to_string(motor.run_on_start ? 1 : 0));
    }

    if (read_status_on_start_) {
      send_line("STATUS");
    }
  }

  std::string normalize_control_mode(const std::string & value) const
  {
    if (value == "manual" || value == "MANUAL" || value == "1") {
      return "MANUAL";
    }
    if (value == "ros" || value == "ROS" || value == "2") {
      return "ROS";
    }
    return "MIXED";
  }

  void read_serial()
  {
    if (fd_ < 0) {
      return;
    }

    char buffer[128];
    while (true) {
      const ssize_t n = read(fd_, buffer, sizeof(buffer));
      if (n > 0) {
        for (ssize_t i = 0; i < n; ++i) {
          const char c = buffer[i];
          if (c == '\r') {
            continue;
          }
          if (c == '\n') {
            publish_status_line();
          } else {
            read_buffer_.push_back(c);
          }
        }
        continue;
      }

      if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_WARN(get_logger(), "Serial read failed: %s", strerror(errno));
      }
      break;
    }
  }

  void publish_status_line()
  {
    if (read_buffer_.empty()) {
      return;
    }

    std_msgs::msg::String msg;
    msg.data = read_buffer_;
    status_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "extruder >> %s", read_buffer_.c_str());
    read_buffer_.clear();
  }

  std::string port_;
  int baud_;
  std::string control_mode_;
  bool read_status_on_start_;
  int fd_{-1};
  std::string read_buffer_;
  std::vector<MotorConfig> motors_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr read_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExtruderSerialNode>());
  rclcpp::shutdown();
  return 0;
}
