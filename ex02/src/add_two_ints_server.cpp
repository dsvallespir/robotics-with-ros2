#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>

using AddTwoInts = example_interfaces::srv::AddTwoInts;

class AddTwoIntsServer: public rclcpp::Node
{
public: 
    AddTwoIntsServer() : Node("add_two_ints_server")
    {
        // Create service
        service_ = create_service<AddTwoInts>(
            "add_two_ints",
            std::bind(&AddTwoIntsServer::handle_service, this, std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "Service ready");
    }

private:
    void handle_service(
        const std::shared_ptr<rmw_request_id_t> request_header;
        const std::shared_ptr<AddTwoInts::Request> request;
        const std::shared_ptr<AddTwoInts::Response> response)
        {
            (void)request_header; // avoid warning

            // Logging
            RCLCPP_INFO(this->get_logger(), "Request received: a=%ld, b=%ld", request->a, request->b);
            RCLCPP_INFO(this->get_logger(), "Sending response: sum=%ld", response->sum);
        }

        rclcpp::Service<AddTwoInts>::SharedPtr service_;
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}