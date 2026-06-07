// Bridge: teleop hand command (/hand_finger_angles) -> Unitree G1 Inspire DDS.
//
// The G1 only listens on the Unitree DDS topic rt/inspire/cmd; it cannot
// subscribe to our perception topics. This node converts our existing
// HandFingerAnglesArray into a unitree_go/MotorCmds (12 entries) and publishes
// it to rt/inspire/cmd. It optionally republishes rt/inspire/state onto a clean
// ROS 2 topic for monitoring.
//
// 12-entry layout (Unitree convention):
//   indices 0..5  = right hand, 6..11 = left hand
//   per-hand order [pinky, ring, middle, index, thumb_bend, thumb_rotation]
//   q normalized: 0 = close, 1 = open
//
// Our /hand_finger_angles *_cmd fields are uint16 0..1000 with 0=open,
// 1000=closed, so per slot: q = 1.0 - cmd/1000.0.

#include <array>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/motor_cmds.hpp"
#include "unitree_go/msg/motor_states.hpp"
#include "hand_perception_msgs/msg/hand_finger_angles_array.hpp"
#include "g1_inspire_hand_bridge/msg/hand_state.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr int kPerHand = 6;
constexpr int kTotal = 12;   // 6 right + 6 left
constexpr float kOpenQ = 1.0f;  // safe pose = fully open

float clamp01(float v)
{
  return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}

// Our cmd: 0=open .. 1000=closed  ->  Unitree q: 0=close .. 1=open
float cmd_to_q(uint16_t cmd)
{
  return clamp01(1.0f - static_cast<float>(cmd) / 1000.0f);
}
}  // namespace

class G1InspireHandBridge : public rclcpp::Node
{
public:
  G1InspireHandBridge() : rclcpp::Node("g1_inspire_hand_bridge_node")
  {
    network_interface_   = declare_parameter<std::string>("network_interface", "");
    ros_command_topic_   = declare_parameter<std::string>("ros_command_topic", "/hand_finger_angles");
    ros_state_topic_     = declare_parameter<std::string>("ros_state_topic", "/g1_inspire/state");
    // The robot's inspire_g1 service (raw Unitree SDK) uses native DDS topic
    // "rt/inspire/cmd". From ROS 2 + CycloneDDS that native topic is reached by
    // publishing to "inspire/cmd" (rmw maps it onto rt/inspire/cmd). Publishing
    // to "rt/inspire/cmd" from ROS would become "/rt/inspire/cmd" natively — a
    // DIFFERENT topic the service never sees.
    dds_cmd_topic_       = declare_parameter<std::string>("dds_cmd_topic", "inspire/cmd");
    dds_state_topic_     = declare_parameter<std::string>("dds_state_topic", "inspire/state");
    publish_state_       = declare_parameter<bool>("publish_state", true);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.5);
    safe_open_on_idle_   = declare_parameter<bool>("safe_open_on_idle", true);
    right_label_         = declare_parameter<std::string>("right_handedness_label", "Right");
    left_label_          = declare_parameter<std::string>("left_handedness_label", "Left");

    // Start from a known-safe pose (all open).
    q_.fill(kOpenQ);

    // Output to the Unitree DDS command topic. BEST_EFFORT to match the robot's
    // inspire_g1 subscriber (which is BEST_EFFORT).
    auto dds_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    cmd_pub_ = create_publisher<unitree_go::msg::MotorCmds>(dds_cmd_topic_, dds_qos);

    // Input from our perception pipeline. BEST_EFFORT to match the publisher.
    auto sensor_qos = rclcpp::SensorDataQoS();
    cmd_sub_ = create_subscription<hand_perception_msgs::msg::HandFingerAnglesArray>(
      ros_command_topic_, sensor_qos,
      std::bind(&G1InspireHandBridge::on_hand_cmd, this, std::placeholders::_1));

    if (publish_state_) {
      state_pub_ = create_publisher<g1_inspire_hand_bridge::msg::HandState>(ros_state_topic_, 10);
      state_sub_ = create_subscription<unitree_go::msg::MotorStates>(
        dds_state_topic_, dds_qos,
        std::bind(&G1InspireHandBridge::on_dds_state, this, std::placeholders::_1));
    }

    // Safety / heartbeat timer (50 Hz).
    last_cmd_time_ = now();
    timer_ = create_wall_timer(20ms, std::bind(&G1InspireHandBridge::on_timer, this));

    // Publish one safe frame so the hands initialize to a known pose.
    publish_cmd();

    RCLCPP_INFO(get_logger(),
      "g1_inspire_hand_bridge ready\n"
      "  in  (ROS)  : %s\n"
      "  out (DDS)  : %s\n"
      "  state out  : %s%s\n"
      "  timeout    : %.2fs  safe_open_on_idle=%s\n"
      "  net iface  : '%s' (DDS handled by the unitree_ros2 bridge)",
      ros_command_topic_.c_str(), dds_cmd_topic_.c_str(),
      publish_state_ ? ros_state_topic_.c_str() : "(disabled)",
      publish_state_ ? "" : "",
      command_timeout_sec_, safe_open_on_idle_ ? "true" : "false",
      network_interface_.c_str());
  }

private:
  // Fill 6 slots [base..base+5] from one HandFingerAngles entry.
  template <typename HandMsg>
  void fill_hand(int base, const HandMsg & h)
  {
    q_[base + 0] = cmd_to_q(h.pinky_cmd);
    q_[base + 1] = cmd_to_q(h.ring_cmd);
    q_[base + 2] = cmd_to_q(h.middle_cmd);
    q_[base + 3] = cmd_to_q(h.index_cmd);
    q_[base + 4] = cmd_to_q(h.thumb_bend_cmd);
    q_[base + 5] = cmd_to_q(h.thumb_rot_cmd);
  }

  void on_hand_cmd(const hand_perception_msgs::msg::HandFingerAnglesArray::SharedPtr msg)
  {
    bool got_right = false, got_left = false;
    for (const auto & h : msg->hands) {
      if (h.handedness == right_label_) {
        fill_hand(0, h);
        got_right = true;
      } else if (h.handedness == left_label_) {
        fill_hand(kPerHand, h);
        got_left = true;
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "unrecognized handedness label '%s' (expected '%s' or '%s')",
          h.handedness.c_str(), right_label_.c_str(), left_label_.c_str());
      }
    }

    if (!got_right && !got_left) {
      return;  // nothing usable; keep previous q_ and let timeout logic handle it
    }

    last_cmd_time_ = now();
    idle_ = false;
    publish_cmd();

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "cmd in: right=%s left=%s | R[%.2f %.2f %.2f %.2f %.2f %.2f] L[%.2f %.2f %.2f %.2f %.2f %.2f]",
      got_right ? "yes" : "no", got_left ? "yes" : "no",
      q_[0], q_[1], q_[2], q_[3], q_[4], q_[5],
      q_[6], q_[7], q_[8], q_[9], q_[10], q_[11]);
  }

  void publish_cmd()
  {
    unitree_go::msg::MotorCmds out;
    out.cmds.resize(kTotal);
    for (int i = 0; i < kTotal; ++i) {
      out.cmds[i].q = q_[i];  // inspire example only consumes q()
    }
    cmd_pub_->publish(out);
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
      "published MotorCmds(12) to %s", dds_cmd_topic_.c_str());
  }

  void on_timer()
  {
    const double dt = (now() - last_cmd_time_).seconds();
    if (dt > command_timeout_sec_) {
      if (!idle_) {
        idle_ = true;
        RCLCPP_WARN(get_logger(),
          "no command for %.2fs (>%.2f): %s", dt, command_timeout_sec_,
          safe_open_on_idle_ ? "sending one safe-open frame, then holding"
                             : "holding last command, not republishing");
        if (safe_open_on_idle_) {
          q_.fill(kOpenQ);
          publish_cmd();
        }
      }
      // While idle we do NOT keep republishing stale commands.
      return;
    }
  }

  void on_dds_state(const unitree_go::msg::MotorStates::SharedPtr msg)
  {
    if (msg->states.size() < static_cast<size_t>(kTotal)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "state has %zu entries, expected %d", msg->states.size(), kTotal);
      return;
    }
    g1_inspire_hand_bridge::msg::HandState out;
    out.header.stamp = now();
    for (int i = 0; i < kPerHand; ++i) {
      out.right_q[i] = msg->states[i].q;
      out.left_q[i] = msg->states[i + kPerHand].q;
    }
    state_pub_->publish(out);
  }

  // params
  std::string network_interface_, ros_command_topic_, ros_state_topic_;
  std::string dds_cmd_topic_, dds_state_topic_, right_label_, left_label_;
  bool publish_state_{true}, safe_open_on_idle_{true};
  double command_timeout_sec_{0.5};

  // state
  std::array<float, kTotal> q_{};
  rclcpp::Time last_cmd_time_;
  bool idle_{false};

  // ros
  rclcpp::Publisher<unitree_go::msg::MotorCmds>::SharedPtr cmd_pub_;
  rclcpp::Subscription<hand_perception_msgs::msg::HandFingerAnglesArray>::SharedPtr cmd_sub_;
  rclcpp::Publisher<g1_inspire_hand_bridge::msg::HandState>::SharedPtr state_pub_;
  rclcpp::Subscription<unitree_go::msg::MotorStates>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<G1InspireHandBridge>());
  rclcpp::shutdown();
  return 0;
}
