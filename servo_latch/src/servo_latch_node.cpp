#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

extern "C" {
#include <pigpiod_if2.h>
}

constexpr int SERVO_PIN   = 18;    // BCM pin for servo signal
constexpr int PULSE_OPEN  = 2000;  // microseconds (roughly 180°)
constexpr int PULSE_CLOSE = 1000;  // microseconds (roughly 0°)

class ServoLatchNode : public rclcpp::Node
{
public:
  ServoLatchNode()
  : Node("servo_latch_node"), pi_handle_(-1)
  {
    using std::placeholders::_1;
    using std::placeholders::_2;

    // Connect to local pigpio daemon (must be running as "pigpiod")
    pi_handle_ = pigpio_start(nullptr, nullptr);
    if (pi_handle_ < 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to connect to pigpiod daemon (pigpio_start returned %d).",
                   pi_handle_);
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Connected to pigpiod daemon, handle = %d", pi_handle_);
      set_mode(pi_handle_, SERVO_PIN, PI_OUTPUT);
    }

    service_ = this->create_service<std_srvs::srv::SetBool>(
      "set_latch",
      std::bind(&ServoLatchNode::handle_set_latch, this, _1, _2));

    RCLCPP_INFO(this->get_logger(),
                "ServoLatchNode is up. Call /set_latch to open/close.");
  }

  ~ServoLatchNode()
  {
    if (pi_handle_ >= 0) {
      // Stop sending pulses and disconnect from daemon
      set_servo_pulsewidth(pi_handle_, SERVO_PIN, 0);
      pigpio_stop(pi_handle_);
      RCLCPP_INFO(this->get_logger(), "Disconnected from pigpiod.");
    }
  }

private:
  void handle_set_latch(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (pi_handle_ < 0) {
      RCLCPP_ERROR(this->get_logger(),
                   "pigpiod connection not available, cannot move servo.");
      response->success = false;
      response->message = "pigpiod connection failed";
      return;
    }

    if (request->data) {
      openLatch();
      response->message = "Latch opened";
    } else {
      closeLatch();
      response->message = "Latch closed";
    }

    response->success = true;
  }

  void openLatch()
  {
    if (pi_handle_ < 0) return;
    set_servo_pulsewidth(pi_handle_, SERVO_PIN, PULSE_OPEN);
    RCLCPP_INFO(this->get_logger(), "Latch OPEN command sent");
  }

  void closeLatch()
  {
    if (pi_handle_ < 0) return;
    set_servo_pulsewidth(pi_handle_, SERVO_PIN, PULSE_CLOSE);
    RCLCPP_INFO(this->get_logger(), "Latch CLOSE command sent");
  }

  int pi_handle_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoLatchNode>());
  rclcpp::shutdown();
  return 0;
}
