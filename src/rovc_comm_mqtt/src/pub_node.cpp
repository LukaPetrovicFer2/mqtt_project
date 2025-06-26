#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <mqtt/async_client.h>
#include <vector>
#include <string>
#include <memory>

class MqttBridgeNode : public rclcpp::Node {
public:
  MqttBridgeNode()
  : Node("mqtt_bridge_node")
  {
    // load MQTT config 
    std::string mqtt_host = this->declare_parameter<std::string>("mqtt_host", "tcp://localhost:1883");
    std::string mqtt_client_id = this->declare_parameter<std::string>("mqtt_client_id", "ros2_mqtt_bridge"); // ip addres defined in params.yaml

    mqtt_client_ = std::make_unique<mqtt::async_client>(mqtt_host, mqtt_client_id); // await connection (add try/catch later for disconnections)
    mqtt_client_->connect()->wait();
    RCLCPP_INFO(this->get_logger(), "✅ Connected to MQTT broker at %s", mqtt_host.c_str()); // confirmation

    // declare and subscribe to each topic group based on data type
    declare_and_subscribe<std_msgs::msg::Float64>("float64_topics", float64_subs_);
    declare_and_subscribe<std_msgs::msg::Float64MultiArray>("float64_multiarray_topics", float64_multiarray_subs_);
    declare_and_subscribe<std_msgs::msg::UInt8>("uint8_topics", uint8_subs_);
  }

private:
  std::unique_ptr<mqtt::async_client> mqtt_client_;

  // declare the message types
  std::vector<rclcpp::SubscriptionBase::SharedPtr> float64_subs_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> float64_multiarray_subs_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> uint8_subs_;

  template<typename MsgType>
  void declare_and_subscribe(const std::string & param_name,
                             std::vector<rclcpp::SubscriptionBase::SharedPtr> & subscriptions)
  {
    std::vector<std::string> mappings = this->declare_parameter(param_name, std::vector<std::string>{});

    for (const auto & entry : mappings) {
      auto delimiter_pos = entry.find(':'); // find the : in params which splits the ROS2 and MQTT topic names (they're the same now)
      if (delimiter_pos == std::string::npos) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Invalid mapping format (expected 'ros_topic:mqtt_topic'): %s", entry.c_str());
        continue;
      }

      std::string ros_topic = entry.substr(0, delimiter_pos);
      std::string mqtt_topic = entry.substr(delimiter_pos + 1);

      auto sub = this->create_subscription<MsgType>(
        ros_topic, 10, // queue size
        [this, mqtt_topic](const typename MsgType::SharedPtr msg) { //forward any declared ROS2 topic to MQTT
          this->publish_to_mqtt<MsgType>(mqtt_topic, msg);
        }
      );
      subscriptions.push_back(sub); // Sub save and confirmation for debbuging (likely removed in final ver.)
      RCLCPP_INFO(this->get_logger(), "📡 Subscribed to ROS topic '%s' → MQTT topic '%s'",
                  ros_topic.c_str(), mqtt_topic.c_str());
    }
  }

  template<typename MsgType>
  void publish_to_mqtt(const std::string & mqtt_topic, const typename MsgType::SharedPtr & msg) {
    std::string payload;

    if constexpr (std::is_same_v<MsgType, std_msgs::msg::Float64>) {
      payload = std::to_string(msg->data); // check msg type and convert to string
    } else if constexpr (std::is_same_v<MsgType, std_msgs::msg::UInt8>) {
      payload = std::to_string(msg->data);
    } else if constexpr (std::is_same_v<MsgType, std_msgs::msg::Float64MultiArray>) { // format multiarray as comma separated string
      for (size_t i = 0; i < msg->data.size(); ++i) { 
        payload += std::to_string(msg->data[i]);
        if (i < msg->data.size() - 1) payload += ",";
      }
    } else {
      RCLCPP_WARN(this->get_logger(), " Unsupported message type for MQTT publishing.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Publishing to MQTT topic '%s': %s", // Pub confirmation for debuggingy  (will be removed in final version)
              mqtt_topic.c_str(), payload.c_str());

    try {
      mqtt_client_->publish(mqtt_topic, payload.c_str(), payload.length(), 1, false); // QoS = 1, false = not retained
    } catch (const mqtt::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "MQTT publish failed: %s", e.what()); // fail msg
    }
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MqttBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
