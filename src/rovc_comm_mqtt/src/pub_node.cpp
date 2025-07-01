#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <mutex>
#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/parameter_client.hpp"

using namespace std::chrono_literals;
using json = nlohmann::json;

class PubNode : public rclcpp::Node
{
public:
    PubNode() : Node("pub_node")
    {
        declare_parameter<std::vector<std::string>>("topics");
        declare_parameter<std::vector<std::string>>("types");

        std::vector<std::string> topics, types;
        get_parameter("topics", topics);
        get_parameter("types", types);

        if (topics.size() != types.size()) {
            RCLCPP_ERROR(this->get_logger(), "topics and types must have the same size");
            return;
        }

        for (size_t i = 0; i < topics.size(); ++i) {
            if (types[i] == "float64")
                subscribe<std_msgs::msg::Float64>(topics[i]);
            else if (types[i] == "float64_multi_array")
                subscribe<std_msgs::msg::Float64MultiArray>(topics[i]);
            else
                RCLCPP_WARN(this->get_logger(), "Unsupported type for topic %s", topics[i].c_str());
        }

        pub_ = this->create_publisher<std_msgs::msg::String>("mqtt_aggregated", 10);
        timer_ = this->create_wall_timer(200ms, std::bind(&PubNode::publish_combined_json, this));
    }

private:
    template<typename MsgType>
    void subscribe(const std::string &topic_name)
    {
        auto callback = [this, topic_name](typename MsgType::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if constexpr (std::is_same_v<MsgType, std_msgs::msg::Float64>)
                data_[topic_name] = static_cast<double>(msg->data);
            else if constexpr (std::is_same_v<MsgType, std_msgs::msg::Float64MultiArray>)
                data_[topic_name] = std::vector<double>(msg->data.begin(), msg->data.end());
        };

        subs_.push_back(this->create_subscription<MsgType>(topic_name, 10, callback));
    }

    void publish_combined_json()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        json j;

        for (const auto &kv : data_)
        {
            j[kv.first] = kv.second;
        }

        auto msg = std_msgs::msg::String();
        msg.data = j.dump();

        pub_->publish(msg);
    }

    std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unordered_map<std::string, json> data_;
    std::mutex data_mutex_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PubNode>());
    rclcpp::shutdown();
    return 0;
}
