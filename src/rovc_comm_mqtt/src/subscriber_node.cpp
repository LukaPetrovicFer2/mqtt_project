#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>
#include <unordered_map>
#include <memory>

using json = nlohmann::json;

class MqttCallback : public virtual mqtt::callback
{
public:
    MqttCallback(std::shared_ptr<rclcpp::Node> node,
                 std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> &publishers)
        : node_(node), publishers_(publishers) {}

    void message_arrived(mqtt::const_message_ptr msg) override
    {
        std::string payload = msg->to_string();
        RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "JSON received: %s", payload.c_str());

        try
        {
            json j = json::parse(payload);

            for (auto it = j.begin(); it != j.end(); ++it)
            {
                const std::string &topic_name = it.key();
                double value = it.value();

                if (publishers_.find(topic_name) == publishers_.end())
                {
                    // Create new publisher on demand
                    auto pub = node_->create_publisher<std_msgs::msg::Float64>(topic_name, 10);
                    publishers_[topic_name] = pub;
                    RCLCPP_INFO(node_->get_logger(), "Created new publisher for topic: %s", topic_name.c_str());
                }

                std_msgs::msg::Float64 msg;
                msg.data = value;
                publishers_[topic_name]->publish(msg);
            }
        }
        catch (const json::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to parse JSON: %s", e.what());
        }
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> &publishers_;
};

class MovementSubscriber : public rclcpp::Node
{
public:
    MovementSubscriber() : Node("subscriber_node"),
                           mqtt_client_("tcp://192.168.2.100:1883", "ros2_subscriber")
    {
        mqtt::connect_options connOpts;
        connOpts.set_clean_session(true);

        try
        {
            mqtt_client_.connect(connOpts)->wait();
            mqtt_client_.subscribe("/tcu_cmds", 1);
            RCLCPP_INFO(this->get_logger(), "Connected to MQTT broker and subscribed to '/tcu_cmds'.");
        }
        catch (const mqtt::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to MQTT broker: %s", e.what());
        }

        mqtt_callback_ = std::make_shared<MqttCallback>(shared_from_this(), publishers_);
        mqtt_client_.set_callback(*mqtt_callback_);
    }

private:
    mqtt::async_client mqtt_client_;
    std::shared_ptr<MqttCallback> mqtt_callback_;
    std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> publishers_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovementSubscriber>());
    rclcpp::shutdown();
    return 0;
}
