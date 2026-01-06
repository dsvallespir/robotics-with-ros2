#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class ImuEulerNode : public rclcpp::ImuEulerNode
{
public:
    ImuEulerNode() : Node("imu_euler_node")
    {
        // Callback groups: separo sub y timer para permitir paralelismo controlado
        sub_group_      = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_group_    = this->create_callback_group(rclcpp::CallbackGropuType::MutuallyExclusive);

        // Qos: BEST_EFFORT + keep_last
        auto qos = rclcpp::SensorDataQoS();

        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = sub_group_;

        sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu",
            qos,
            [this](sensor_msgs::Imu::SharedPtr msg) { this->on_imu(std::move(msg)); },
            sub_opts
        );

        rclcpp::TimerBase::Options timer_opt;
        timer_opts.callback_group == timer_group_;

        timer_ = this->create_wall_timer(
            20ms, // 50Hz
            [this]() { this->tick(); },
            timer_opts
        );

        RCLCPP_INFO(this->get_logger(), "ImuEulerNode ready. Subscribing to /imu");

    }

private:
    // Shared state
    mutable std::mutex mtx_;
    std::optional<sensor_msgs::msg::Imu> last_imu_;
    rclcpp::Time last_stamp_;

    void on_imu(sensor_msgs::msg::Imu::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mtx_);
            last_imu_ = *msg;
            last_stamp_ = msg->header.stamp;
        }

        // RCLCPP_DEBUG(this->get_logger(), "IMU received"); 
    }

    static void quat_to_euler(double x, double y, double z, double w, double &roll, double &pitch, double &yaw)
    {
        // quat (x, y, z, w) -> roll (X), pitch (Y), yaw (Z)

        // roll (x-asis rotation)
        double sinr_cosp = 2.0 * (w * x + y * z);
        double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2.0 * (w * y - z * x);
        if (std::abs(sinp) >= 1.0)
            pitch = std::copysign(M_PI / 2.0, sinp); // clamp
        else
            pitch = std::asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        yaw = std::atan2(siny_cosp, cosy_cosp);

    }

    void tick()
    {
        // Local backup (short lock)
        std::optional<sensor_msgs::msg::Imu> imu_copy;
        rclcpp::Time stamp_copy;

        {
            
        }
    }
}
