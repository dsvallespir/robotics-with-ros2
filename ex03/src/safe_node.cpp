#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SafeNode : public rclcpp::Node
{
public:
    SafeNode() : Node("safe_node")
    {
        sub_group_      = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_group_    = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubcriptionOptions sub_opts;
        sub_opts.callback_group = sub_group_;

        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic",
            rclcpp::QoS(10),
            [this](std_msgs::msg::String::SharedPtr msg) { this->on_msg(std::move(msg)); },
            sub_opts
        );

        rclcpp::TimerBase::Options timer_opts;
        timer_opts.callback_group = timer_group_;

        timer_ = this->create_wall_timer(
            20ms, // 50 Hz
            [this]() { this->control_loop(); },
            timer_opts
        );

        RCLCPP_INFO(this->get_logger(), "SafeNode ready (MultiThread friendly).");
    }

private:
    // Estado compartido (protegido por mutex)
    mutable std::mutex mtx_;
    std::string  last_msg_;
    uint64_t seq_{0};

    void on_msg(std_msgs::msg::String::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mtx_);
            last_msg_ = msg->data;
            seq_++;
        }

        RCLCPP_INFO(this->get_logger(), "RX msg: '%s'", msg->data.c_str());
    }

    void conrtol_loop()
    {
        std::string_msg_copy;
        uint64_t seq_copy = 0;

        {
            std::lock_guard<std::mutex> lock(mtx_);
            msg_copy = last_msg_;
            seq_copy = seq_;
        }

        // Fuera del lock

        RCLCPP_INFO(this->get_logger(), "Ctrl tick: seq=%lu last'%s'", static_cast<unsigned long>(seq_copy), msg_copy.c_str());
    }

    rclcpp::CallbackGroup::SharedPtr sub_group_;
    rclcpp::CallbackGroup::SharedPtr timer_group_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SafeNode>();

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}