#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <mqtt/async_client.h>

class MqttCallback : public virtual mqtt::callback
{
public:
    MqttCallback(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher)
        : publisher_(publisher) {}

    void message_arrived(mqtt::const_message_ptr msg) override
    {
        std::string payload = msg->to_string();

        // Log
        RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "Move: %s", payload.c_str());

        // Convert to ROS2 message and publish
        auto ros_message = std_msgs::msg::String();
        ros_message.data = payload;
        publisher_->publish(ros_message);
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

class MovementSubscriber : public rclcpp::Node
{
public:
    MovementSubscriber() : Node("subscriber_node"),
                           mqtt_client_("tcp://192.168.2.100:1883", "ros2_subscriber") // Use the Windows server IP
    {
        // ROS2 Publisher to republish MQTT messages
        publisher_ = this->create_publisher<std_msgs::msg::String>("movement_command", 10);

        // MQTT Setup
        mqtt::connect_options connOpts;
        connOpts.set_clean_session(true);

        try
        {
            mqtt_client_.connect(connOpts)->wait();
            mqtt_client_.subscribe("movement_command", 1);
            RCLCPP_INFO(this->get_logger(), "Connected to MQTT broker and subscribed to 'movement_command'.");
        }
        catch (const mqtt::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to MQTT broker: %s", e.what());
        }

        // Attach callback for MQTT messages
        mqtt_callback_ = std::make_shared<MqttCallback>(publisher_);
        mqtt_client_.set_callback(*mqtt_callback_);
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    mqtt::async_client mqtt_client_;
    std::shared_ptr<MqttCallback> mqtt_callback_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovementSubscriber>());
    rclcpp::shutdown();
    return 0;
}
