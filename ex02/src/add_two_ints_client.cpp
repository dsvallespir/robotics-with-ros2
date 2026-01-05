#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;
using AddTwoInts = example_interfaces::srv::AddTwoInts;

class AddTwoIntsCliente : public rclcpp::Node
{
public:
    AddToIntsClient() : Node("add_two_ints_client")
    {
        // Create client
        client_ = create_client<AddTwoInts>("add_two_ints");

        // Waiting for service ready
        while (!client_->wait_for_service(1s)){
            if (!rclcpp::ok) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting service");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Waiting service...");
        }

        // Send request
        send_request();
    }

    void send_request()
    {
        auto request = std::make_shared<AddTwoInts::Request>();
        request->a = 5;
        request->b = 7;

        // Send asynchronous request
        auto future = client_->async_send_request(request);

        // Wait for response (synchronous version)
        if (rclcpp::spin_until_future_complete(shared_from_this(), future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "sum: %ld", future.get()->sum);            
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed calling the service");
        }
    }

private:
    rclcpp::Client<AddTwoInts>::SharedPtr client_;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddToIntsClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}