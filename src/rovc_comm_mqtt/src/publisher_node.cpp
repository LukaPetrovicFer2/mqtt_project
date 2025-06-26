#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include <mqtt/async_client.h>
#include <chrono>
#include <thread>
#include <random>

using namespace std::chrono_literals;

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode() : Node("publisher_node"),
                      mqtt_client_("tcp://192.168.2.100:1883", "ros2_publisher"), // Must match the
                      is_connected_(false),                                      // the order in
                      altitude_(generate_random_altitude())                     // private section
    {
        // ROS2 Publishers (add relevant topics)
        publisher_ = this->create_publisher<std_msgs::msg::String>("ros2/mqtt/topic", 10);
        altitude_publisher_ = this->create_publisher<std_msgs::msg::Float32>("ros2/mqtt/altitude", 10);

        // Last will and testament
        mqtt::connect_options connOpts;
        connOpts.set_clean_session(true);
        connOpts.set_will(mqtt::message("ros2/mqtt/topic", "ROS2 disconnected!", 1, false));

        // MQTT Setup
        connect_to_broker();

        // Timer to publish messages every x seconds
        timer_ = this->create_wall_timer(3s, std::bind(&PublisherNode::publish_messages, this));
    }

    ~PublisherNode()
    {
        if (mqtt_client_.is_connected())
        {
            // Send a disconnect message before shutting down
            mqtt::message_ptr disconnect_msg = mqtt::make_message("ros2/mqtt/topic", "ROS2 disconnected!");
            disconnect_msg->set_qos(1);
            mqtt_client_.publish(disconnect_msg)->wait();

            RCLCPP_INFO(this->get_logger(), "Sent disconnect message to MQTT broker.");
            mqtt_client_.disconnect()->wait();
        }
    }

private:
    mqtt::async_client mqtt_client_; // same declaration order
    bool is_connected_; //              as at the start 
    float altitude_; //                 of PublisherNode

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr altitude_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void connect_to_broker()
    {
        mqtt::connect_options connOpts;
        connOpts.set_clean_session(true);
        connOpts.set_will(mqtt::message("ros2/mqtt/topic", "ROS2 disconnected!", 1, false));

        while (!is_connected_)
        {
            try
            {
                mqtt_client_.connect(connOpts)->wait();
                is_connected_ = true;
                RCLCPP_INFO(this->get_logger(), "Connected to MQTT broker.");
            }
            catch (const mqtt::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to connect to MQTT broker: %s", e.what());
                std::this_thread::sleep_for(5s); // Wait and retry
            }
        }
    }

    void publish_messages()
    {
        if (!mqtt_client_.is_connected())
        {
            RCLCPP_WARN(this->get_logger(), "MQTT broker disconnected! Attempting to reconnect...");
            is_connected_ = false;
            connect_to_broker(); // Attempt to reconnect
        }

        // **Publish ROS Data String**
        auto message = std_msgs::msg::String();
        message.data = "ROS2 node is running!";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);

        // Publish to MQTT Topic
        mqtt::message_ptr mqtt_msg = mqtt::make_message("ros2/mqtt/topic", message.data);
        mqtt_msg->set_qos(1);
        
        try
        {
            mqtt_client_.publish(mqtt_msg)->wait();
            RCLCPP_INFO(this->get_logger(), "Published to topic 'ros2/mqtt/topic'");
        }
        catch (const mqtt::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to publish to MQTT: %s", e.what());
            is_connected_ = false; // Mark as disconnected
        }

        // **Publish Altitude Value**
        update_altitude(); // Update altitude with 50/50 chance to change

        auto altitude_msg = std_msgs::msg::Float32();
        altitude_msg.data = altitude_;
        RCLCPP_INFO(this->get_logger(), "Publishing Altitude: %.2f", altitude_msg.data);
        altitude_publisher_->publish(altitude_msg);

        // Publish to MQTT Altitude Topic
        mqtt::message_ptr mqtt_altitude_msg = mqtt::make_message("ros2/mqtt/altitude", std::to_string(altitude_));
        mqtt_altitude_msg->set_qos(1);
        try
        {
            mqtt_client_.publish(mqtt_altitude_msg)->wait();
            RCLCPP_INFO(this->get_logger(), "Published to topic 'ros2/mqtt/altitude'");
        }
        catch (const mqtt::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to publish Altitude to MQTT: %s", e.what());
            is_connected_ = false; // Mark as disconnected
        }
    }

    void update_altitude()
    {
        // Generate a random number (0, 1, or 2) to determine altitude change
        int change = rand() % 3; // 0 = no change, 1 = increase, 2 = decrease

        if (change == 1 && altitude_ < 1000) // Increase
        {
            altitude_ += 1;
        }
        else if (change == 2 && altitude_ > 0) // Decrease
        {
            altitude_ -= 1;
        }
        // 50% chance of no change (if `change == 0`)
    }

    float generate_random_altitude()
    {
        // Initialize random seed and generate a float between 0 and 1000
        srand(time(0));
        return static_cast<float>(rand() % 1001);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherNode>());
    rclcpp::shutdown();
    return 0;
}