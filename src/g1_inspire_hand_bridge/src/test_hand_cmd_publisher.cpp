// Test publisher: emits HandFingerAnglesArray (both hands) on the bridge input
// topic so you can verify bridge -> rt/inspire/cmd without the camera.
//
//   ros2 run g1_inspire_hand_bridge test_hand_cmd_publisher --ros-args
//        -p mode:=close          (open | close | half | cycle)
//        -p topic:=/hand_finger_angles
//
// Our cmd convention: 0 = open, 1000 = closed.

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "hand_perception_msgs/msg/hand_finger_angles_array.hpp"

using namespace std::chrono_literals;

class TestHandCmdPublisher : public rclcpp::Node
{
public:
  TestHandCmdPublisher() : rclcpp::Node("test_hand_cmd_publisher")
  {
    mode_  = declare_parameter<std::string>("mode", "half");
    topic_ = declare_parameter<std::string>("topic", "/hand_finger_angles");

    pub_ = create_publisher<hand_perception_msgs::msg::HandFingerAnglesArray>(
      topic_, rclcpp::SensorDataQoS());
    timer_ = create_wall_timer(100ms, std::bind(&TestHandCmdPublisher::tick, this));

    RCLCPP_INFO(get_logger(), "publishing mode='%s' on '%s' (0=open,1000=closed)",
                mode_.c_str(), topic_.c_str());
  }

private:
  uint16_t value_for_mode()
  {
    if (mode_ == "open")  return 0;
    if (mode_ == "close") return 1000;
    if (mode_ == "half")  return 500;
    if (mode_ == "cycle") {
      // 0 -> 1000 -> 0 triangle wave, ~4 s period
      phase_ = (phase_ + 1) % 80;
      int v = phase_ < 40 ? phase_ * 25 : (80 - phase_) * 25;
      return static_cast<uint16_t>(v);
    }
    return 500;
  }

  hand_perception_msgs::msg::HandFingerAngles make_hand(const std::string & label, uint16_t v)
  {
    hand_perception_msgs::msg::HandFingerAngles h;
    h.handedness = label;
    h.score = 1.0f;
    h.index_cmd = h.middle_cmd = h.ring_cmd = h.pinky_cmd = v;
    h.thumb_bend_cmd = h.thumb_rot_cmd = v;
    return h;
  }

  void tick()
  {
    const uint16_t v = value_for_mode();
    hand_perception_msgs::msg::HandFingerAnglesArray msg;
    msg.header.stamp = now();
    msg.hands.push_back(make_hand("Right", v));
    msg.hands.push_back(make_hand("Left", v));
    pub_->publish(msg);
  }

  std::string mode_, topic_;
  int phase_{0};
  rclcpp::Publisher<hand_perception_msgs::msg::HandFingerAnglesArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestHandCmdPublisher>());
  rclcpp::shutdown();
  return 0;
}
