#include "my_lifecycle_component.hpp"
#include <memory>
#include <chrono>

MyLifecycleComponent::MyLifecycleComponent(const std::string& node_name)
: LifecycleNode(node_name), count_(0), resource_initialized_(false)
{
    this->declare_parameter("publish_rate", 1.0);
    this->declare_parameter("topic_name", "status");
    this->declare_parameter("node_description", "Lifecycle Component Example");

    RCLCPP_INFO(this->get_logger(),
                "Constructor: %s creado (estado: %s)",
                node_name.c_str(),
                this->get_current_state().label().c_str());

}

// Callback: Configuring

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MyLifecycleComponent::on_configure(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(this->get_logger(),
                "on_configure llamado desde estado: %s",
                previous_state.label().c_str());

    try {
        // 1. Get parameters
        double publish_rate = this->get_parameter("publish_rate").as_double();
        std::string topic_name = this->get_parameter("topic_name").as_string();

        // 2. Create publisher (but no activete yet)
        publisher_ = this-> create_publisher<std_msgs::msg:String>(topic_name, 10);

        // 3. Create timer
        auto timer_duration = std::chrono::duration<double>(1.0 / publish_rate);
        timer_ = this->create_wall_timer(timer_duration, std::bind(&MyLifecycleComponent::timer_callback, this));

        // 4. Initialize weight resources
        initializeResources();

        // 5. Configure QoS and other parameters
        RCLCPP_INFO(this->get_logger(),
                    "Configured with rate: %.1f Hz, topic: %s",
                    publish_rate, topic_name.c_str());

        resource_initialized_ = true;

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(),
                    "Error en configuración: %s", e.what());
        
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

}

//Callback: ACTIVATING
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MyLifecycleComponent::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(this->get_logger(),
                "on_activate llamado desde estado: %s"),
                previous_state.label().c_str());

    // Activate publisher (only on ACTIVE state can publish messages)
    publisher_->on_activate();

    // Initialize active processing
    timer_->reset(); // Initialize timer

    RCLCPP_INFO(this->get_logger(), "ACTIVATE Component")
}

//Callback: DEACTIVATING
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MyLifecycleComponent::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    
}

//Callback: CLEANING UP
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MyLifecycleComponent::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
    
}

//Callback: SHUTTING DOWN
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MyLifecycleComponent::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
    
}