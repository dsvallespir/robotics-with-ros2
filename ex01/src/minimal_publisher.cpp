#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
    public:
        MinimalPublisher() : Node("minimal_publisher"), count_(0)
        {
            // Create publisher with QoS
            publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

            // Create timer
            timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this)));

            RCLCPP_INFO(this->get_logger(), "Publisher created");
        }
    private:
        void timer_callback()
        {
            auto message = std_msgs::msg::String();
            message.data = "Hello, ROS 2 C++!" + std::to_string(count_++);

            // Publicar mensaje
            publisher_->publish(message);

            // Logging with different levels
            RCLCPP_INFO(this->get_logger(), "Publicacindo: '%s'", message.data.c_str());
            // RCLCPP_DEBUG, RCLCPP_WARN, RCLCPP_ERROR 

            if (count_ % 10 == 0) {
                RCLCPP_WARN(this->get_logger(), "Publishing %ld messages", count_);
            }
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
    
}

int main(int argc, char * argv[])
{
    // Inicializar ROS 2
    rclcpp::init(argc, argv);

    // Crear y ejecutar nodo
    auto node = std::make_shared<MinimalPublisher>();

    // Spin del nodo
    rclcpp::sping(node);

    // Limpieza
    rclcpp::shutdown();
    return 0;
}